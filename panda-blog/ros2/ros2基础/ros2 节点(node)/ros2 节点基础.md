# ROS2 节点基础

## 什么是ROS2节点？

ROS2节点（Node）是ROS2系统中的基本计算单元，每个节点都是一个独立的进程，负责执行特定的功能。

### 节点的特点
- **独立性**：每个节点都是独立的进程
- **模块化**：不同功能分解到不同节点
- **分布式**：可以运行在不同机器上
- **可重用性**：可以在不同系统中重复使用

## 节点类的概念

节点类是一个继承自ROS2 Node基类的Python类，它定义了节点的具体行为。

### 节点类的作用
- **封装功能**：把节点的所有功能封装在一个类里
- **定义行为**：指定节点启动时和运行时做什么
- **管理状态**：保存节点运行过程中需要的数据

## 创建节点的逻辑流程

### 伪代码展示
```python
# 1. 定义节点类
class 我的节点(Node):
    def __init__(self):
        # 设置节点名称
        # 创建需要的组件（定时器、订阅者等）
        # 输出启动信息
    
    def 定时任务(self):
        # 定期执行的任务
        # 比如输出心跳信息

# 2. 主函数流程
def main():
    # 初始化ROS2系统
    # 创建节点实例
    # 启动节点运行
    # 处理异常
    # 清理资源

# 3. 程序入口
if __name__ == '__main__':
    main()
```

### 关键逻辑点
1. **节点类定义**：继承Node基类，实现自己的功能
2. **初始化设置**：在`__init__`中设置节点名称和创建组件
3. **主函数管理**：负责启动、运行、关闭整个流程
4. **异常处理**：确保节点能够优雅地启动和关闭

## 创建节点的基本要素

### 1. 节点类（Node Class）
继承自ROS2 Node基类的Python类，定义节点的具体行为。

### 2. 节点名称
每个节点必须有唯一的名称，用于在ROS2系统中标识节点。

### 3. 初始化函数
在节点启动时执行，设置节点的基本配置。

### 4. 主函数
节点的入口点，负责启动和运行节点。

### 5. 生命周期管理
节点有明确的启动、运行和关闭过程。

---

## 代码实现

```python
#!/usr/bin/env python3
"""
最简单的ROS2节点示例

这个节点演示了ROS2节点的基本结构：
1. 导入必要的ROS2库
2. 创建节点类
3. 初始化节点
4. 运行节点
"""

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    """
    最简单的ROS2节点类
    
    这个类继承自Node基类，实现了最基本的节点功能：
    - 节点初始化
    - 基本的日志输出
    - 定时器回调
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建定时器，定期执行回调函数
        3. 输出初始化日志
        """
        # 调用父类构造函数，设置节点名称为 'simple_node'
        # 节点名称在ROS2系统中必须是唯一的
        super().__init__('simple_node')
        
        # 输出初始化日志
        self.get_logger().info('节点已启动')
        
        # 创建定时器，每1秒执行一次timer_callback函数
        # 定时器是ROS2中常用的机制，用于周期性执行任务
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        """
        定时器回调函数
        
        这个函数会被定时器定期调用，用于：
        - 输出心跳信息
        - 演示节点的持续运行
        """
        # 输出心跳信息
        self.get_logger().info('节点正在运行')


def main(args=None):
    """
    主函数 - 节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建节点实例
    3. 运行节点
    4. 清理资源
    """
    # 初始化ROS2客户端库
    # 这是使用ROS2功能的必要步骤
    rclpy.init(args=args)
    
    # 创建SimpleNode实例
    simple_node = SimpleNode()
    
    # 运行节点，开始处理回调
    # spin()函数会让节点持续运行，直到被中断
    rclpy.spin(simple_node)
    
    # 关闭ROS2客户端库
    rclpy.shutdown()


if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行这个Python文件时，会执行main()函数
    这是Python的标准做法
    """
    main()
```

## 代码解析

### 节点类实现
```python
class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
```
**对应伪代码**：`class 我的节点(Node):` 和 `# 设置节点名称`
- 继承Node基类
- 设置节点名称为'simple_node'

### 初始化函数
```python
def __init__(self):
    super().__init__('simple_node')
    self.get_logger().info('节点已启动')
    self.timer = self.create_timer(1.0, self.timer_callback)
```
**对应伪代码**：`# 创建需要的组件（定时器、订阅者等）` 和 `# 输出启动信息`
- 设置节点名称
- 创建定时器（节点的具体功能）
- 输出初始化日志

### 主函数
```python
def main(args=None):
    rclpy.init(args=args)
    simple_node = SimpleNode()
    rclpy.spin(simple_node)
    rclpy.shutdown()
```
**对应伪代码**：主函数流程中的各个步骤
- `rclpy.init()`：初始化阶段
- `rclpy.spin()`：运行阶段
- `rclpy.shutdown()`：清理阶段

## 节点的生命周期

### 1. 初始化阶段
- 导入库
- 初始化ROS2系统
- 创建节点实例

### 2. 运行阶段
- 处理回调
- 执行任务
- 持续运行

### 3. 清理阶段
- 销毁节点
- 关闭ROS2系统

## 关键概念

### 节点名称
每个节点必须有唯一的名称，用于在ROS2系统中标识节点。

### 日志系统
使用`self.get_logger()`输出日志信息。

### 定时器
用于周期性执行任务，可以设置执行频率。
