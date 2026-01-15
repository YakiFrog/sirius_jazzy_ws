# Sirius System Diagram (Academic/Paper Version)

This version is optimized for white-background publications (papers, reports). It features large text and high contrast for maximum readability.

```mermaid
%%{init: {
  'theme': 'base', 
  'themeVariables': { 
    'fontSize': '24px',
    'fontFamily': 'arial',
    'primaryColor': '#ffffff', 
    'primaryTextColor': '#000000', 
    'primaryBorderColor': '#000000', 
    'lineColor': '#000000', 
    'tertiaryColor': '#ffffff', 
    'clusterBkg': '#ffffff', 
    'clusterBorder': '#000000', 
    'titleColor': '#000000',
    'edgeLabelBackground':'#ffffff'
  }
}}%%
graph LR
    subgraph HW ["Hardware Drivers"]
        WIMU[("witmotion_node")]
        VELO[("velodyne_node")]
        URG[("urg_node")]
        MCTRL[("roboteq_driver")]
    end

    subgraph Core ["Processing Layer"]
        EKF[("ekf_filter")]
        VSCAN[("velodyne_scan")]
        RSP[("state_publisher")]
        TFT[("TF Tree")]
    end

    subgraph Nav ["Navigation Layer"]
        AMCL[("amcl")]
        NAV2[("nav2_stack")]
        MAP[("map_server")]
        MOVE[("move_goal")]
    end

    %% Data Flow
    WIMU -- "/imu" --> EKF
    VELO -- "/points" --> VSCAN & NAV2
    URG -- "/scan" --> NAV2
    MCTRL -- "/odom" --> EKF

    RSP -- "static_tf" --> TFT
    EKF -- "filtered_odom" --> NAV2
    EKF -- "odom->base" --> TFT
    AMCL -- "map->odom" --> TFT
    VSCAN -- "/scan(2.5D)" --> AMCL & NAV2

    TFT -- "tf" --> NAV2 & MOVE
    MAP -- "/map" --> AMCL & NAV2
    
    MOVE -- "goal" --> NAV2
    MOVE -- "/stop" --> MCTRL
    
    NAV2 -- "/cmd_vel" --> MCTRL

    %% Minimalistic styles
    style HW fill:#eee,stroke:#000,stroke-width:2px
    style Core fill:#eee,stroke:#000,stroke-width:2px
    style Nav fill:#eee,stroke:#000,stroke-width:2px
    style TFT fill:#fff,stroke:#000,stroke-width:2px
```

## 論文・レポート用設定
- **高コントラスト**: 白背景に黒文字・黒線で、印刷時やPDF閲覧時の視認性を最大化しています。
- **特大フォント**: `fontSize: 24px` に設定し、論文内で図を縮小して配置しても文字が潰れないように工夫しています。
- **ノード名称の簡略化**: スペースを節約しつつ正確性を保つため、プロセスの実行ファイル名に近い名称を採用しています。
- **モノクロスタイル**: 色を多様せず、薄いグレーの塗りつぶしのみを使用することで、上品でアカデミックな外観にしています。
