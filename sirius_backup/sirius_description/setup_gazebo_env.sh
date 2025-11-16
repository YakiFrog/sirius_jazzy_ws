#!/bin/bash

# Sirius3 DescriptionパッケージのGazebo環境変数設定

# パッケージのパスを取得
SIRIUS_DESCRIPTION_PATH=$(ros2 pkg prefix sirius_description)/share/sirius_description

# Gazeboのモデルパスを設定
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$SIRIUS_DESCRIPTION_PATH/sdf

echo "Gazebo environment for sirius_description configured."
echo "GZ_SIM_RESOURCE_PATH: $GZ_SIM_RESOURCE_PATH"
