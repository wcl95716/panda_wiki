# VPN 服务器搭建教程

## [Trojan VPN 搭建](https://github.com/Jrohy/trojan)

### 功能特性

- 多用户管理（Web界面 + 命令行）
- 流量统计和限制
- 自动SSL证书申请
- 客户端配置生成

### 快速安装

```bash
# 一键安装/更新
source <(curl -sL https://git.io/trojan-install)

# 卸载
source <(curl -sL https://git.io/trojan-install) --remove
```

安装后：

- 输入 `trojan` 进入管理程序
- 访问 `https://域名` 进行Web管理

### Docker 安装

```bash
# 1. 安装数据库
docker run --name trojan-mariadb --restart=always -p 3306:3306 \
  -v /home/mariadb:/var/lib/mysql \
  -e MYSQL_ROOT_PASSWORD=trojan -e MYSQL_ROOT_HOST=% \
  -e MYSQL_DATABASE=trojan -d mariadb:10.2

# 2. 安装trojan
docker run -it -d --name trojan --net=host --restart=always --privileged jrohy/trojan init

# 3. 初始化配置
docker exec -it trojan bash
# 在容器内输入: trojan

# 4. 启动服务
systemctl start trojan-web
systemctl enable trojan-web
```

### 常用命令

```bash
trojan add            # 添加用户
trojan del            # 删除用户
trojan info           # 用户列表
trojan start/stop/restart/status  # 服务管理
trojan log            # 查看日志
trojan tls            # 证书管理
trojan update         # 更新程序
```

---

## [网络加速](https://github.com/jinwyp/one_click_script)

### 一键安装

```bash
bash <(curl -Lso- https://git.io/oneclick)
```

---

## BBR 网络加速

### 内核安装选项


| 系统                   | 推荐选项 | 内核版本 |
| ------------------------ | ---------- | ---------- |
| CentOS/AlmaLinux/Rocky | 35       | LTS 5.10 |
| Debian                 | 41       | LTS 5.10 |
| Ubuntu                 | 45       | LTS 5.10 |

### 安装步骤

1. **安装内核**: 运行脚本选择对应选项
2. **重启系统**: 安装过程中会重启两次
3. **启用BBR**: 重新运行脚本选择 **2** 启用BBR + Cake

### BBR Plus 选项

- **61**: BBR Plus 4.14.129 内核
- **66**: BBR Plus 5.10 LTS 内核
- 安装后选择 **3** 启用BBR Plus

### 加速效果

- ✅ 提升网络传输速度
- ✅ 降低网络延迟
- ✅ 提高连接稳定性

> ⚠️ **注意**: 安装过程中如遇警告屏幕，选择"否"继续，不要中止内核删除
