# ROS2 话题基础

## 什么是ROS2话题？

ROS2话题（Topic）是ROS2中节点间通信的基本方式，采用**发布-订阅**模式。发布者（Publisher）发送消息，订阅者（Subscriber）接收消息，两者通过话题名称进行连接。

### 话题的特点
- **异步通信**：发布者和订阅者不需要同时运行
- **一对多**：一个话题可以有多个发布者和订阅者
- **类型安全**：消息有固定的数据类型
- **松耦合**：发布者和订阅者互不依赖

## 话题通信的概念

### 发布者（Publisher）
负责向话题发送消息的节点。

### 订阅者（Subscriber）
负责从话题接收消息的节点。

### 消息（Message）
话题传输的数据，有固定的数据类型。

## 创建话题通信的逻辑流程

### 伪代码展示
```python
# 1. 发布者节点
class 发布者节点(Node):
    def __init__(self):
        # 设置节点名称
        # 创建发布者
        # 创建定时器，定期发送消息
    
    def 发送消息(self):
        # 创建消息
        # 发布消息

# 2. 订阅者节点
class 订阅者节点(Node):
    def __init__(self):
        # 设置节点名称
        # 创建订阅者
    
    def 接收消息(self, 消息):
        # 处理接收到的消息

# 3. 主函数流程
def main():
    # 初始化ROS2系统
    # 创建节点实例
    # 运行节点
    # 清理资源
```

### 关键逻辑点
1. **发布者**：创建发布者，定期发送消息
2. **订阅者**：创建订阅者，定义消息处理函数
3. **消息类型**：使用标准消息类型（如String）
4. **话题名称**：发布者和订阅者通过相同的话题名称连接

## 创建话题通信的基本要素

### 1. 发布者（Publisher）
负责向话题发送消息的组件。

### 2. 订阅者（Subscriber）
负责从话题接收消息的组件。

### 3. 消息类型
话题传输的数据类型，如String、Int32等。

### 4. 话题名称
发布者和订阅者连接的话题标识。

### 5. 回调函数
订阅者处理接收消息的函数。

## 代码实现

### 发布者节点
```python
#!/usr/bin/env python3
"""
发布者节点示例

这个节点演示了如何创建发布者：
1. 导入必要的ROS2库和消息类型
2. 创建发布者节点类
3. 定期发布消息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherNode(Node):
    """
    发布者节点类
    
    这个类继承自Node基类，实现了发布者功能：
    - 创建发布者
    - 定期发送消息
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建发布者
        3. 创建定时器，定期发送消息
        """
        # 调用父类构造函数，设置节点名称为 'publisher_node'
        super().__init__('publisher_node')
        
        # 创建发布者，发布String类型的消息到'topic'话题
        # 10是队列大小，用于缓存消息
        self.publisher = self.create_publisher(String, 'topic', 10)
        
        # 创建定时器，每1秒执行一次timer_callback函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 输出初始化日志
        self.get_logger().info('发布者节点已启动')
    
    def timer_callback(self):
        """
        定时器回调函数
        
        这个函数会被定时器定期调用，用于：
        - 创建消息
        - 发布消息到话题
        """
        # 创建String类型的消息
        msg = String()
        msg.data = f'Hello World! 时间戳: {self.get_clock().now()}'
        
        # 发布消息到话题
        self.publisher.publish(msg)
        
        # 输出发布日志
        self.get_logger().info(f'已发布消息: {msg.data}')


def main(args=None):
    """
    主函数 - 发布者节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建发布者节点实例
    3. 运行节点
    4. 清理资源
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    
    # 创建PublisherNode实例
    publisher_node = PublisherNode()
    
    # 运行节点，开始处理回调
    rclpy.spin(publisher_node)
    
    # 关闭ROS2客户端库
    rclpy.shutdown()


if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行这个Python文件时，会执行main()函数
    """
    main()
```

### 订阅者节点
```python
#!/usr/bin/env python3
"""
订阅者节点示例

这个节点演示了如何创建订阅者：
1. 导入必要的ROS2库和消息类型
2. 创建订阅者节点类
3. 处理接收到的消息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    """
    订阅者节点类
    
    这个类继承自Node基类，实现了订阅者功能：
    - 创建订阅者
    - 处理接收到的消息
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建订阅者
        """
        # 调用父类构造函数，设置节点名称为 'subscriber_node'
        super().__init__('subscriber_node')
        
        # 创建订阅者，订阅'topic'话题的String类型消息
        # callback是消息处理函数
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        
        # 输出初始化日志
        self.get_logger().info('订阅者节点已启动，正在监听话题')
    
    def listener_callback(self, msg):
        """
        消息处理回调函数
        
        这个函数会在接收到消息时被调用，用于：
        - 处理接收到的消息
        - 输出消息内容
        """
        # 输出接收到的消息
        self.get_logger().info(f'收到消息: {msg.data}')


def main(args=None):
    """
    主函数 - 订阅者节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建订阅者节点实例
    3. 运行节点
    4. 清理资源
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    
    # 创建SubscriberNode实例
    subscriber_node = SubscriberNode()
    
    # 运行节点，开始处理回调
    rclpy.spin(subscriber_node)
    
    # 关闭ROS2客户端库
    rclpy.shutdown()


if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行这个Python文件时，会执行main()函数
    """
    main()
```

## 代码解析

### 发布者实现
```python
# 创建发布者
self.publisher = self.create_publisher(String, 'topic', 10)

# 发送消息
msg = String()
msg.data = 'Hello World!'
self.publisher.publish(msg)
```
**对应伪代码**：`# 创建发布者` 和 `# 发布消息`
- 创建String类型的发布者
- 设置话题名称为'topic'
- 创建消息并发布

### 订阅者实现
```python
# 创建订阅者
self.subscription = self.create_subscription(
    String, 'topic', self.listener_callback, 10
)

# 消息处理函数
def listener_callback(self, msg):
    self.get_logger().info(f'收到消息: {msg.data}')
```
**对应伪代码**：`# 创建订阅者` 和 `# 处理接收到的消息`
- 创建String类型的订阅者
- 订阅'topic'话题
- 定义消息处理回调函数

## 话题通信的生命周期

### 1. 初始化阶段
- 导入消息类型
- 创建发布者或订阅者
- 设置话题名称

### 2. 运行阶段
- 发布者：定期发送消息
- 订阅者：监听并处理消息

### 3. 清理阶段
- 销毁节点
- 关闭ROS2系统

## 关键概念

### 话题名称
发布者和订阅者通过相同的话题名称进行连接。

### 消息类型
话题传输的数据类型，如String、Int32、Float64等。

### 队列大小
用于缓存消息的队列长度，防止消息丢失。

### 回调函数
订阅者处理接收消息的函数，在收到消息时自动调用。
