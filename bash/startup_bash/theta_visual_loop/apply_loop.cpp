#include <karto_sdk/Mapper.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 10) {
    std::cerr << "usage: apply_loop graph_prefix output [first last dx dy yaw sigma_xy sigma_yaw]\n";
    return 2;
  }
  const bool optimize = argc == 10;
  fs::path output = argv[2];
  fs::create_directories(output);
  const std::string prefix = argv[1];
  try {
    karto::Dataset dataset;
    dataset.LoadFromFile(prefix + ".data");
    for (auto * object : dataset.GetLasers()) {
      auto * laser = dynamic_cast<karto::Sensor *>(object);
      if (laser) {
        karto::SensorManager::GetInstance()->RegisterSensor(laser, true);
      }
    }
    karto::Mapper mapper;
    mapper.LoadFromFile(prefix + ".posegraph");
    auto * graph = mapper.GetGraph();
    const auto scans = mapper.GetAllProcessedScans();
    if (scans.empty()) {
      throw std::runtime_error("No serialized scans");
    }
    if (optimize) {
      const int first_id = std::stoi(argv[3]);
      const int last_id = std::stoi(argv[4]);
      const karto::Pose2 measurement(std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7]));
      const double sigma_xy = std::stod(argv[8]);
      const double sigma_yaw = std::stod(argv[9]);
      if (!(sigma_xy > 0.0 && sigma_yaw > 0.0)) {
        throw std::runtime_error("Invalid loop covariance");
      }
      karto::LocalizedRangeScan * first = nullptr;
      karto::LocalizedRangeScan * last = nullptr;
      for (auto * scan : scans) {
        if (scan->GetUniqueId() == first_id) {first = scan;}
        if (scan->GetUniqueId() == last_id) {last = scan;}
      }
      if (!first || !last || first == last) {
        throw std::runtime_error("Missing loop endpoints");
      }

      rclcpp::init(0, nullptr);
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("visual_loop_offline");
      node->declare_parameter("debug_logging", true);
      node->declare_parameter("ceres_trust_strategy", "LEVENBERG_MARQUARDT");
      node->declare_parameter("ceres_preconditioner", "SCHUR_JACOBI");
      pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox", "karto::ScanSolver");
      auto solver = loader.createSharedInstance("solver_plugins::CeresSolver");
      solver->Configure(node);
      for (const auto & sensor : graph->GetVertices()) {
        for (const auto & vertex : sensor.second) {
          solver->AddNode(vertex.second);
        }
      }
      for (auto * edge : graph->GetEdges()) {
        if (edge) {solver->AddConstraint(edge);}
      }

      kt_bool new_edge = false;
      auto * edge = graph->AddEdge(first, last, new_edge);
      if (!edge || !new_edge) {
        throw std::runtime_error("Could not add new visual loop edge");
      }
      karto::Matrix3 covariance;
      covariance.SetToIdentity();
      covariance(0, 0) = covariance(1, 1) = sigma_xy * sigma_xy;
      covariance(2, 2) = sigma_yaw * sigma_yaw;
      edge->SetLabel(new karto::LinkInfo(karto::Pose2(), measurement, covariance));
      solver->AddConstraint(edge);
      solver->Compute();
      const auto & corrections = solver->GetCorrections();
      if (corrections.size() != scans.size()) {
        throw std::runtime_error("Optimizer did not return every node");
      }
      std::map<int, karto::Pose2> corrected(corrections.begin(), corrections.end());
      for (auto * scan : scans) {
        const auto & pose = corrected.at(scan->GetUniqueId());
        if (!std::isfinite(pose.GetX()) || !std::isfinite(pose.GetY()) ||
          !std::isfinite(pose.GetHeading()))
        {
          throw std::runtime_error("Nonfinite optimized pose");
        }
        scan->SetCorrectedPoseAndUpdate(pose);
      }
      mapper.SaveToFile((output / "posegraph.posegraph").string());
      fs::copy_file(prefix + ".data", output / "posegraph.data", fs::copy_options::overwrite_existing);
      rclcpp::shutdown();
    }

    std::ofstream poses(output / "poses.csv");
    poses << "id,stamp,x,y,yaw\n" << std::setprecision(17);
    std::ofstream points(output / "endpoint_points.csv");
    points << "id,x,y,range\n" << std::setprecision(10);
    std::ofstream endpoint_ranges(output / "endpoint_ranges.csv");
    endpoint_ranges << "id,index,range\n" << std::setprecision(17);
    for (size_t i = 0; i < scans.size(); ++i) {
      auto * scan = scans[i];
      const auto pose = scan->GetCorrectedPose();
      poses << scan->GetUniqueId() << ',' << scan->GetTime() << ',' << pose.GetX() << ','
            << pose.GetY() << ',' << pose.GetHeading() << '\n';
      if (i == 0 || i + 1 == scans.size()) {
        const auto & ranges = scan->GetRangeReadingsVector();
        for (size_t j = 0; j < ranges.size(); ++j) {
          endpoint_ranges << scan->GetUniqueId() << ',' << j << ',' << ranges[j] << '\n';
        }
      }
      if (i >= 24 && i + 32 < scans.size()) {continue;}
      const auto sensor_pose = scan->GetSensorPose();
      for (const auto & p : scan->GetPointReadings(true)) {
        const double range = std::hypot(p.GetX() - sensor_pose.GetX(), p.GetY() - sensor_pose.GetY());
        if (std::isfinite(range) && range >= 1.0 && range <= 35.0) {
          points << scan->GetUniqueId() << ',' << p.GetX() << ',' << p.GetY() << ',' << range << '\n';
        }
      }
    }
    std::ofstream edges(output / "edges.csv");
    edges << "source,target,dx,dy,yaw\n" << std::setprecision(17);
    for (auto * edge : graph->GetEdges()) {
      auto * label = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
      if (!label) {throw std::runtime_error("Graph edge has no measurement");}
      const auto delta = label->GetPoseDifference();
      edges << edge->GetSource()->GetObject()->GetUniqueId() << ','
            << edge->GetTarget()->GetObject()->GetUniqueId() << ',' << delta.GetX() << ','
            << delta.GetY() << ',' << delta.GetHeading() << '\n';
    }

    constexpr double resolution = 0.05;
    std::unique_ptr<karto::OccupancyGrid> grid(karto::OccupancyGrid::CreateFromScans(
      scans, resolution, mapper.getParamMinPassThrough(), mapper.getParamOccupancyThreshold()));
    if (!grid) {throw std::runtime_error("Empty occupancy map");}
    std::ofstream pgm(output / "map.pgm", std::ios::binary);
    pgm << "P5\n" << grid->GetWidth() << ' ' << grid->GetHeight() << "\n255\n";
    for (int y = grid->GetHeight() - 1; y >= 0; --y) {
      for (int x = 0; x < grid->GetWidth(); ++x) {
        const auto cell = *grid->GetDataPointer(karto::Vector2<kt_int32s>(x, y));
        pgm.put(cell == karto::GridStates_Occupied ? 0 : cell == karto::GridStates_Free ? 254 : 205);
      }
    }
    const auto origin = grid->GetCoordinateConverter()->GetOffset();
    std::ofstream meta(output / "map.yaml");
    meta << std::setprecision(17)
         << "image: map.pgm\nmode: trinary\nresolution: " << resolution
         << "\norigin: [" << origin.GetX() << ", " << origin.GetY()
         << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    if (!pgm || !meta || !poses || !points || !edges || !endpoint_ranges) {
      throw std::runtime_error("Failed writing results");
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
