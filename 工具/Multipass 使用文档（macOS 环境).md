# Multipass 使用指南

Multipass 是一个用于快速创建 Ubuntu 虚拟机（类似 CT 容器体验）的工具，支持 macOS、Windows 和 Linux，适合开发者轻量级使用。

---

## ✅ 安装 Multipass

macOS 安装方式：

```bash
brew install --cask multipass
```

安装完成后，GUI 工具也会一并安装，可通过搜索 "Multipass" 启动图形界面。

---

## 🚀 快速创建虚拟机

```bash
multipass launch -n devbox
```

默认：Ubuntu LTS、1 CPU、1G 内存、5G 磁盘

自定义资源：

```bash
multipass launch -n ros2 -c 4 -m 8G -d 55G
```

示例参数说明：

* `-n`: 名称
* `-c`: CPU 数量
* `-m`: 内存大小（支持 MiB/GiB）
* `-d`: 磁盘大小（支持 GiB）

---

## 🔍 查看可用系统版本

```bash
multipass find
```

列出如 `22.04`, `20.04`, `24.04` 等可用 Ubuntu 镜像。

---

## 📦 查看和管理实例

```bash
multipass list       # 查看实例
multipass info devbox  # 查看 devbox 实例详细信息
```

---

## 📁 Cloud-init 自定义配置

你可以通过 `cloud-init.yaml` 配置实例初始化行为，例如创建用户、添加 SSH 密钥、安装软件：

cloud-init.yaml 示例：

```yaml
#cloud-config
users:
  - name: panda
    ssh_authorized_keys:
      - ssh-ed25519 AAAAC3N... panda@MacBook-Pro-2.local
    sudo: ["ALL=(ALL) NOPASSWD:ALL"]
    groups: sudo
    shell: /bin/bash
packages:
  - git
  - curl
runcmd:
  - echo "Hello from cloud-init" > /home/panda/welcome.txt
```

使用 cloud-init 创建实例：

```bash
multipass launch -n custombox --cloud-init cloud-init.yaml
```

---

## 🔑 使用公钥 SSH 登录

Multipass 实例默认通过当前系统用户的公钥认证（路径通常为 `~/.ssh/id_ed25519.pub`）。

你也可以手动将公钥传入 cloud-init，如上所示，或手动添加：

```bash
multipass transfer ~/.ssh/id_ed25519.pub devbox:
multipass shell devbox
mkdir -p ~/.ssh && cat id_ed25519.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys
```

在 `.ssh/config` 中添加：

```bash
Host devbox
  HostName 192.168.x.x
  User panda
  IdentityFile ~/.ssh/id_ed25519
```

然后即可：

```bash
ssh devbox
```

---

## 🖥 图形化使用 GUI

Multipass 安装后默认附带图形界面，可以：

* 选择镜像版本
* 拖动设置 CPU/内存/磁盘
* 配置桥接网络（支持访问宿主机局域网）

建议勾选：`Bridged network: ON` 可直接通过 IP 访问。

---

## ❓ 常见问题

### Q1. 如何设置桥接网络以获得真实局域网 IP？

A: GUI 创建时打开 `Connect to bridged network` 或使用 `--bridged en0` 参数（取决于你网卡名称）

### Q2. 为何 VSCode SSH 无法连接？

A: 请确保 `~/.ssh/id_ed25519.pub` 已写入目标虚拟机的 `~/.ssh/authorized_keys` 中，且权限设置正确。

### Q3. 无法设置密码怎么办？

A: Multipass 默认只支持 key 登录，建议通过 cloud-init 添加用户与密码或公钥。

---

## ✅ 总结

Multipass 提供了轻量、快速、近似 LXC 的虚拟机体验，适合开发调试、云端迁移、快速部署。

你可以将它作为 Proxmox VE 的补充工具，用于 macOS 桌面环境下的开发测试场景。
