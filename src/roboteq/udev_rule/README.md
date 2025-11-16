# Udevルールをしようね
## Roboteq用のUdevルールを作成する
```bash
code /etc/udev/rules.d/
```
内容は同じディレクトリにある
## 最後にリロード
```bash
sudo udevadm control --reload
```