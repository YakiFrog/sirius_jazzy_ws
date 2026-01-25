# Sirius System Diagram (Academic/Paper Version)

This version is optimized for white-background publications (papers, reports). It features large text and high contrast for maximum readability.

```mermaid
%%{init: {
  'theme': 'base', 
  'themeVariables': { 
    'fontSize': '32px',
    'fontFamily': 'arial',
    'primaryColor': '#ffffff', 
    'primaryTextColor': '#000000', 
    'primaryBorderColor': '#000000', 
    'lineColor': '#000000', 
    'tertiaryColor': '#ffffff', 
    'clusterBkg': '#ffffff', 
    'clusterBorder': '#000000', 
    'titleColor': '#000000',
    'edgeLabelBackground':'#ffffff',
    'nodeSpacing': 100,
    'rankSpacing': 150
  }
}}%%
graph LR
    subgraph HW ["Sensors & Actuators"]
        SENSORS["LiDAR / IMU"]
        DRIVE["Motor Driver"]
    end

    subgraph CORE ["Processing & Navigation"]
        EKF["EKF Fusion"]
        NAV2["Navigation (Nav2)"]
        MAP["Map Server"]
    end

    %% Data Flow
    SENSORS -- "/scan, /imu" --> CORE
    DRIVE -- "/odom" --> EKF
    
    EKF -- "Filtered Odom" --> NAV2
    MAP -- "/map" --> NAV2
    
    NAV2 -- "/cmd_vel" --> DRIVE

    %% Minimalistic styles
    style HW fill:#eee,stroke:#000,stroke-width:3px
    style CORE fill:#fff,stroke:#000,stroke-width:3px
    style SENSORS fill:#fff,stroke:#000,stroke-width:2px
    style DRIVE fill:#fff,stroke:#000,stroke-width:2px
    style EKF fill:#fff,stroke:#000,stroke-width:2px
    style NAV2 fill:#fff,stroke:#000,stroke-width:2px
    style MAP fill:#fff,stroke:#000,stroke-width:2px
```

## 論文・レポート用設定
- **高コントラスト**: 白背景に黒文字・黒線で、印刷時やPDF閲覧時の視認性を最大化しています。
- **特大フォント**: `fontSize: 24px` に設定し、論文内で図を縮小して配置しても文字が潰れないように工夫しています。
- **ノード名称の簡略化**: スペースを節約しつつ正確性を保つため、プロセスの実行ファイル名に近い名称を採用しています。
- **モノクロスタイル**: 色を多様せず、薄いグレーの塗りつぶしのみを使用することで、上品でアカデミックな外観にしています。
