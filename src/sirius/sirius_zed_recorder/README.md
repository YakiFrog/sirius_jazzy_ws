# Sirius ZED Recorder

実機ロボットPC上で、ZEDの補正済みSide-by-Side画像と工場キャリブレーションを
ROS 2へ配信するCPU専用パッケージです。CUDAとZED SDKは使用しません。

## 対応機種

- USB接続のZED、ZED Mini、ZED 2、ZED 2i
- ZED X、ZED X Mini、ZED X OneなどのGMSL機種は非対応

カメラ取得には公式の
[ZED Open Capture](https://github.com/stereolabs/zed-open-capture)を使用し、
ビルド時に固定リビジョンを取得します。初回ビルドおよびカメラごとの工場
キャリブレーション取得時のみインターネット接続が必要です。

## セットアップと起動

```bash
bash ~/sirius_jazzy_ws/bash/startup_bash/setup_offline_real.sh
source ~/sirius_jazzy_ws/install/setup.bash
ros2 launch sirius_zed_recorder zed_stereo_publisher.launch.py
```

既定値は片眼1280x720、15 FPS、JPEG品質90です。CPUやディスク帯域が不足する場合は
`config/zed_offline_recorder.yaml`のFPSまたはJPEG品質を下げてください。

## 出力

- `/camera/stereo_sbs/compressed`: 左右を横連結した歪み補正済みJPEG
- `/camera/stereo_params`: `fx`, `fy`, `cx`, `cy`, `baseline`などのJSON

工場キャリブレーションは`~/zed/settings/SN<シリアル>.conf`へキャッシュされます。
録画前には`check_offline_real`を実行し、画像レートとSLAM Toolboxの補正済みTFを
確認してください。録画プログラム自身はセンサーやSLAM Toolboxを自動起動しません。
