# Multipass 使用手册

Multipass 是由 Canonical 提供的轻量级 VM 管理工具，特别适合快速启动 Ubuntu 虚拟机。

---

## 📦 安装

### macOS

```bash
brew install --cask multipass
```

---

## 🚀 创建与管理虚拟机

### 创建虚拟机（默认 Ubuntu 22.04）

```bash
multipass launch --name devbox
```

### 指定系统版本、CPU、内存、磁盘大小

```bash
multipass launch --name devbox \
  --cpus 2 \
  --mem 2G \
  --disk 10G \
  22.04
```

---

## 🛠 管理虚拟机

### 查看虚拟机信息

```bash
multipass info devbox
```

### 查看所有实例

```bash
multipass list
```

### 登录虚拟机

```bash
multipass shell devbox
```

### 删除虚拟机

```bash
multipass delete devbox
multipass purge  # 清理残留数据
```

---

## 📡 网络信息

创建的虚拟机会自动分配私有 IPv4 地址：

```bash
multipass info devbox
# IPv4: 192.168.x.x
```

你可以通过此 IP 用 SSH 工具连接，例如：

```bash
ssh ubuntu@192.168.64.11
```

---

## 🔐 SSH 公钥登录支持

Multipass 默认会将你主机上的 `~/.ssh/id_rsa.pub` 或 `~/.ssh/id_ed25519.pub` 自动注入虚拟机。

### 手动注入本机 SSH 公钥（用于已存在的实例）

```bash
multipass exec devbox -- bash -c "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys" < ~/.ssh/id_ed25519.pub
```

---

## ☁️ 使用 cloud-init 文件初始化虚拟机

### 示例 `cloud-init.yaml`

```yaml
#cloud-config
users:
  - default
  - name: ubuntu
    ssh-authorized-keys:
      - ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIGhzxRKkAyam3hMbPzKPlbpeKV04t9EbYEBMT+2++eOE panda@MacBook-Pro-2.local
    sudo: ['ALL=(ALL) NOPASSWD:ALL']
    shell: /bin/bash
```

### 使用该配置创建虚拟机

```bash
multipass launch --name devbox --cloud-init cloud-init.yaml
```

---

## 📋 查看可用的镜像版本

```bash
multipass find
```

---

## 🖥 是否有图形界面？

Multipass 原生不带 GUI，但可使用以下替代方案：

* [Multipass Tray](https://github.com/canonical/multipass-tray)：非官方图形界面

---

## 📎 其他常用命令

### 文件上传/下载

```bash
multipass transfer ./localfile.txt devbox:/home/ubuntu/
```

### 启动 / 停止

```bash
multipass stop devbox
multipass start devbox
```

### 重命名实例

```bash
multipass alias devbox newname
```

---

如需更多帮助，可运行：

```bash
multipass --help
```
