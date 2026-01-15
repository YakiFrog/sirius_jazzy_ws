# Sirius System Diagram (Real Robot)

This diagram represents the node interactions when running `sirius_jazzy_ws` on the real hardware (Jazzy/ROS 2).

```mermaid
%%{init: {'theme': 'base', 'themeVariables': { 'primaryColor': '#000000', 'primaryTextColor': '#ffffff', 'primaryBorderColor': '#ffffff', 'lineColor': '#ffffff', 'tertiaryColor': '#ffffff', 'clusterBkg': '#222222', 'clusterBorder': '#666666', 'titleColor': '#ffffff'}}}%%
graph TD
    subgraph HW ["Drivers (20-200Hz)"]
        Roboteq[("Roboteq")]
        URG[("URG")]
        Velo[("Velodyne")]
        IMU[("IMU")]
    end

    subgraph Core ["Processing & Fusion (20Hz)"]
        EKF[("EKF")]
        VScan[("VeloScan")]
        RSP[("Robot State Publisher (URDF)")]
    end

    subgraph Nav ["Navigation & Mission (1-10Hz)"]
        Nav2[("Nav2 Stack")]
        AMCL[("AMCL")]
        Map[("Map Server")]
        Move[("Move Goal")]
    end

    subgraph UI ["User Interface"]
        RViz[("RViz2 / UI")]
    end

    %% Data Flow
    Roboteq -- "/odom (20Hz)" --> EKF
    IMU -- "/imu (200Hz)" --> EKF
    URG -- "/hokuyo_scan (10Hz)" --> Nav2
    Velo -- "/velodyne_points (20Hz)" --> VScan
    Velo -- "/velodyne_points (20Hz)" --> Nav2
    VScan -- "/scan3 (20Hz)" --> AMCL
    VScan -- "/scan3 (20Hz)" --> Nav2

    RSP -- "/tf (Static)" --> TfTree[("TF Tree")]
    EKF -- "/tf (20Hz)" --> TfTree
    AMCL -- "/tf (5Hz)" --> TfTree

    TfTree -- "Odom (20Hz)" --> AMCL
    TfTree -- "Pose (2-20Hz)" --> Move
    TfTree -- "Transforms" --> Nav2 & RViz

    Map -- "/map" --> AMCL & Nav2 & RViz
    Move -- "Goal Action" --> Nav2
    Move -- "/target_odom" --> RViz
    Move -- "Change Map" --> Map

    RViz -- "/initialpose" --> AMCL & EKF
    RViz -- "/goal_pose" --> Nav2

    %% Control Flow
    Nav2 -- "/cmd_vel (10Hz)" --> Roboteq
    Move -- "/stop" --> Roboteq

    %% Styles
    style HW fill:#330000,stroke:#f99,color:#fff
    style Core fill:#002200,stroke:#9f9,color:#fff
    style Nav fill:#000033,stroke:#99f,color:#fff
    style UI fill:#222,stroke:#aaa,color:#fff
    style TfTree fill:#444,stroke:#fff,color:#fff
```

## ノードの詳細説明

- **Roboteq**: 20Hzでオドメトリを配信、10Hzで速度指令を受信。
- **URG**: 約10Hzで周辺スキャンを配信。
- **Velodyne**: 約20Hzで3D点群を配信。
- **IMU**: 200Hzの高周期で姿勢情報を配信。
- **VeloScan**: 20Hzで3D点群を仮想2.5Dスキャンに変換。
- **EKF**: 20Hzでセンサーを統合。
- **AMCL**: 5Hz程度（移動時）で自己位置を補正。
- **Nav2**: 10Hzの制御ループで動作。
- **Move Goal**: 0.5Hz（2秒周期）で到達判定と次目標送信。
