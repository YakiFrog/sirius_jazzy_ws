#!/bin/bash
set -e

echo "=== Sirius 32GB Swap Configuration Script ==="
echo "This script will turn off the current swap, resize it to 32GB, and turn it back on."
echo "Root privileges are required. Please enter your password when prompted."
echo ""

# 1. スワップを一時的にオフにする
echo "1. Turning off existing swap..."
sudo swapoff /swap.img

# 2. 新しい32GBのファイルを作成する
echo "2. Allocating 32GB swap space (this may take a few seconds)..."
sudo fallocate -l 32G /swap.img || {
    echo "fallocate failed, falling back to dd (this might take longer)..."
    sudo dd if=/dev/zero of=/swap.img bs=1M count=32768 status=progress
}

# 3. 適切なパーミッションを設定する
echo "3. Setting secure permissions..."
sudo chmod 600 /swap.img

# 4. スワップ領域を構築する
echo "4. Setting up swap area..."
sudo mkswap /swap.img

# 5. スワップを有効化する
echo "5. Activating 32GB swap..."
sudo swapon /swap.img

# 6. 結果確認
echo "=== Complete! Current Memory and Swap Status ==="
free -h
swapon --show
