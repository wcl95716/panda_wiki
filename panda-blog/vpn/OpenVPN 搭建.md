# OpenVPN 搭建

## 配置步骤

### 1. 虚拟机配置
- 选择Ubuntu虚拟机
- 创建成功后设置网络

### 2. 网络设置
- 添加入站规则：目标端口范围设为 `*`
- 添加出站规则：目标端口范围设为 `*`

### 3. 防火墙关闭
```bash
sudo iptables -F  # 清空防火墙规则
sudo iptables -X  # 删除所有自定义链
sudo iptables -P INPUT ACCEPT  # 将输入流量的默认策略设置为允许
sudo iptables -P FORWARD ACCEPT  # 将转发流量的默认策略设置为允许
sudo iptables -P OUTPUT ACCEPT  # 将输出流量的默认策略设置为允许
```

### 4. 运行安装脚本
```bash
wget https://git.io/vpn -O openvpn-install.sh && bash openvpn-install.sh
```

### 5. 配置文件下载
安装完成后，OpenVPN配置文件会自动下载到本地，可以直接使用OpenVPN客户端连接。

## client-to-client 配置

### 1. 编辑配置文件
打开OpenVPN服务器配置文件：
```bash
sudo nano /etc/openvpn/server.conf
```

### 2. 添加配置选项
在配置文件中添加以下行：
```
client-to-client
```

### 3. 重启服务
```bash
sudo service openvpn restart
```

**注意**：确保在进行任何更改之前备份OpenVPN服务器的配置文件，以防止意外的配置错误。 