---
date: "2025-07-01T12:14:44+0800"
author: panda
categories: ["技术文档", "工具使用"]
title: Proxmox
lastmod: "2025-07-05T02:52:44+0000"
---

# Proxmox VE 使用指南

## 简介
Proxmox VE (Virtual Environment) 是一个开源的服务器虚拟化平台，它集成了 KVM 虚拟机和 LXC 容器，提供了一个完整的虚拟化解决方案。Proxmox VE 基于 Debian Linux，并提供了一个易于使用的 Web 界面来管理虚拟机和容器。

## 主要特性
- 基于 Web 的管理界面
- 支持 KVM 虚拟机和 LXC 容器
- 实时迁移功能
- 高可用性集群
- 备份和恢复功能
- 支持多种存储类型（本地、NFS、Ceph、iSCSI等）
- 内置防火墙
- 支持多种网络配置

## 系统要求
- 64位处理器（支持硬件虚拟化）
- 最小 4GB RAM（推荐 8GB 或更多）
- 至少 32GB 存储空间
- 网络连接

## 安装步骤
1. 下载 Proxmox VE ISO 镜像
   - 访问 [Proxmox 下载页面](https://www.proxmox.com/en/downloads)
   - 选择最新版本的 ISO 镜像

2. 创建安装介质
   - 使用工具（如 Rufus）将 ISO 写入 USB 驱动器
   - 确保使用 DD 模式写入

3. 安装过程
   - 将 USB 驱动器插入目标服务器
   - 从 USB 启动
   - 按照安装向导进行操作
   - 设置 root 密码和网络配置

4. 安装后配置
   - 访问 Web 界面：https://服务器IP:8006
   - 使用 root 账户登录
   - 配置存储和网络

## 基本使用
### 创建虚拟机
1. 在 Web 界面中点击"创建 VM"
2. 选择操作系统类型
3. 配置硬件参数（CPU、内存、存储等）
4. 上传或选择安装镜像
5. 完成创建并启动虚拟机

### 创建容器
1. 在 Web 界面中点击"创建 CT"
2. 选择容器模板
3. 配置资源限制
4. 设置网络参数
5. 完成创建并启动容器

### 备份管理
1. 选择要备份的虚拟机或容器
2. 点击"备份"
3. 选择备份存储位置
4. 设置备份计划（可选）
5. 执行备份

## 集群管理
1. 添加节点到集群
2. 配置共享存储
3. 设置高可用性
4. 管理集群资源

## 安全建议
- 定期更新系统
- 使用强密码
- 配置防火墙规则
- 启用双因素认证
- 定期备份数据

## 故障排除
- 检查系统日志
- 验证网络连接
- 确认存储状态
- 检查资源使用情况

## 资源链接
- [官方文档](https://pve.proxmox.com/wiki/Main_Page)
- [社区论坛](https://forum.proxmox.com/)
- [GitHub 仓库](https://github.com/proxmox)

## 许可证
Proxmox VE 是开源软件，基于 GNU Affero General Public License v3 发布。 