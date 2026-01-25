# Sirius System Diagram (Real Robot)

This diagram represents the node interactions when running `sirius_jazzy_ws` on the real hardware (Jazzy/ROS 2).

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'primaryColor': '#000000', 'primaryTextColor': '#ffffff', 'primaryBorderColor': '#ffffff', 'lineColor': '#ffffff', 'tertiaryColor': '#ffffff', 'clusterBkg': '#222222', 'clusterBorder': '#666666', 'titleColor': '#ffffff'}}}%%
graph LR
    subgraph HW ["Hardware Drivers"]
        witmotion_ros_node[("witmotion_ros_node (200Hz)")]
        velodyne_driver_node[("velodyne_driver_node (20Hz)")]
        urg_node2[("urg_node2 (10Hz)")]
        roboteq_ros2_driver[("roboteq_ros2_driver (Actuator)")]
    end

    subgraph Core ["Processing & Sensor Fusion"]
        ekf_filter_node[("ekf_filter_node (20Hz)")]
        velodyne_laserscan_node[("velodyne_laserscan_node (20Hz)")]
        robot_state_publisher[("robot_state_publisher")]
        TfTree[("TF Tree")]
    end

    subgraph Nav ["Navigation & Mission"]
        amcl[("amcl (Dynamic)")]
        nav2_stack[("Nav2 Stack (10Hz)")]
        map_server[("map_server")]
        move_goal[("move_goal")]
    end

    subgraph UI ["User Interface"]
        rviz2[("rviz2")]
    end

    %% Drivers to Processing/Fusion
    witmotion_ros_node -- "/imu (200Hz)" --> ekf_filter_node
    velodyne_driver_node -- "/velodyne_points (20Hz)" --> velodyne_laserscan_node & nav2_stack
    urg_node2 -- "/hokuyo_scan (10Hz)" --> nav2_stack
    roboteq_ros2_driver -- "/odom (20Hz)" --> ekf_filter_node

    %% Internal Processing Flows
    robot_state_publisher -- "/tf (Static)" --> TfTree
    ekf_filter_node -- "/odom/filtered (20Hz)" --> nav2_stack
    ekf_filter_node -- "/tf (20Hz: odom->base)" --> TfTree
    amcl -- "/tf (Dynamic: map->odom)" --> TfTree
    velodyne_laserscan_node -- "/scan3 (20Hz)" --> amcl & nav2_stack

    %% Navigation Inputs
    TfTree -- "/tf" --> nav2_stack & rviz2 & move_goal
    TfTree -- "/tf (odom->base)" --> amcl
    map_server -- "/map (Latch)" --> amcl & nav2_stack & rviz2
    
    %% Mission & UI
    move_goal -- "Action: navigate_to_pose" --> nav2_stack
    move_goal -- "/stop (Event)" --> roboteq_ros2_driver
    move_goal -- "/target_odom" --> rviz2
    rviz2 -- "/initialpose" --> amcl & ekf_filter_node
    rviz2 -- "/goal_pose" --> nav2_stack

    %% Control Output
    nav2_stack -- "/cmd_vel (10Hz)" --> roboteq_ros2_driver

    %% Styles
    style HW fill:#330000,stroke:#f99,color:#fff
    style Core fill:#002200,stroke:#9f9,color:#fff
    style Nav fill:#000033,stroke:#99f,color:#fff
    style UI fill:#222,stroke:#aaa,color:#fff
    style TfTree fill:#444,stroke:#fff,color:#fff
```

## ノードの詳細説明

- **roboteq_ros2_driver**: 20Hzでオドメトリ配信、10Hzで速度指令受信。
- **urg_node2**: 約10Hzで `/hokuyo_scan` を配信。
- **velodyne_driver_node**: 約20Hzで `/velodyne_points` を配信。
- **witmotion_ros_node**: 200Hzの高周期で `/imu` を配信。
- **velodyne_laserscan_node**: 20Hzで点群を `/scan3` に変換。
- **ekf_filter_node**: 20Hzでセンサを統合し `/odom/filtered` を出力。
- **robot_state_publisher**: URDFから静的な `/tf` を配信。
- **amcl**: 移動時のみ補正を実施。
- **nav2_stack**: 10Hz周期でパス追従と制御を実施。
- **move_goal**: 2秒周期でウェイポイント巡回を管理。
