#!/bin/bash
# Sirius Assisted Teleop 完全起動スクリプト
# このスクリプトはNav2を含む全てを起動します

set -e  # エラーで停止

echo "=========================================="
echo "Sirius Assisted Teleop - 完全起動"
echo "=========================================="
echo ""

# カラー定義
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ワークスペースのパス
WS_PATH="/home/kotantu-desktop/sirius_jazzy_ws"

# ワークスペースをソース
echo -e "${YELLOW}ワークスペースをソース中...${NC}"
source "${WS_PATH}/install/setup.bash"
echo -e "${GREEN}✓ 完了${NC}"
echo ""

# シミュレーションが起動しているか確認
echo -e "${YELLOW}シミュレーションの確認中...${NC}"
if ros2 topic list | grep -q "/scan"; then
    echo -e "${GREEN}✓ シミュレーションが起動しています${NC}"
else
    echo -e "${RED}✗ シミュレーションが起動していません${NC}"
    echo ""
    echo "別のターミナルで以下を実行してください:"
    echo "  ros2 launch sirius_description sim.launch.py"
    echo ""
    read -p "シミュレーションを起動しましたか？ (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "起動をキャンセルしました"
        exit 1
    fi
fi
echo ""

# 起動方法を選択
echo "=========================================="
echo "起動方法を選択してください:"
echo "=========================================="
echo "1. Nav2込みで起動（推奨）"
echo "   - Nav2スタック全体を起動"
echo "   - Assisted Teleopを起動"
echo "   - Teleopキーボードを起動"
echo ""
echo "2. Assisted Teleopのみ起動"
echo "   - Nav2が既に起動している場合"
echo ""
read -p "選択 (1 or 2): " -n 1 -r
echo ""
echo ""

if [[ $REPLY == "1" ]]; then
    echo -e "${YELLOW}Nav2込みで起動します...${NC}"
    echo ""
    echo "起動内容:"
    echo "  - Nav2 AMCL (ローカライゼーション)"
    echo "  - Nav2 Behavior Server (Assisted Teleop含む)"
    echo "  - Nav2 Controller Server"
    echo "  - Assisted Teleop Node"
    echo "  - Teleop Keyboard"
    echo ""
    echo -e "${YELLOW}起動まで約10-15秒かかります...${NC}"
    echo ""
    sleep 2
    
    ros2 launch sirius_navigation assisted_teleop_with_nav2.launch.py
    
elif [[ $REPLY == "2" ]]; then
    echo -e "${YELLOW}Nav2の確認中...${NC}"
    
    # Nav2が起動しているか確認
    if ros2 action list | grep -q "assisted_teleop"; then
        echo -e "${GREEN}✓ Nav2が起動しています${NC}"
        echo ""
        echo -e "${YELLOW}Assisted Teleopを起動します...${NC}"
        sleep 2
        
        ros2 launch sirius_navigation assisted_teleop.launch.py
    else
        echo -e "${RED}✗ Nav2が起動していません${NC}"
        echo ""
        echo "Nav2を起動するには、別のターミナルで以下を実行:"
        echo "  ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true"
        echo ""
        echo "または、オプション1を選択してNav2込みで起動してください。"
        exit 1
    fi
else
    echo -e "${RED}無効な選択です${NC}"
    exit 1
fi
