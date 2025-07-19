# ROS2 服务基础

## 什么是服务（Service）？

ROS2服务（Service）是ROS2中节点间通信的另一种方式，采用**请求-响应**模式。服务端（Server）提供功能，客户端（Client）请求服务，客户端发送请求后等待服务端返回响应。

### 服务的特点
- **同步通信**：客户端发送请求后必须等待响应
- **一对一**：一个请求对应一个响应
- **类型安全**：请求和响应都有固定的数据类型
- **功能导向**：适合提供具体的功能服务

## 服务通信的概念

### 服务端（Server）
提供服务的节点，接收请求并返回响应。

### 客户端（Client）
请求服务的节点，发送请求并接收响应。

### 服务类型（Service Type）
定义请求和响应数据结构的类型。

**服务类型结构说明：**
ROS2服务类型使用特定的格式定义，包含请求和响应两部分：
```
int64 a      # 请求参数
int64 b      # 请求参数
---          # 分隔线：分隔请求和响应
int64 sum    # 响应参数
```

- **分隔线（---）**：将服务定义分为请求部分和响应部分
- **请求部分**：客户端发送给服务端的数据
- **响应部分**：服务端返回给客户端的数据

## 创建服务通信的逻辑流程

### 伪代码展示
```python
# 1. 服务端节点
class 服务端节点(Node):
    def __init__(self):
        # 设置节点名称
        # 创建服务端
    
    def 处理请求(self, 请求, 响应):
        # 处理客户端请求
        # 设置响应数据
        # 返回响应

# 2. 客户端节点
class 客户端节点(Node):
    def __init__(self):
        # 设置节点名称
        # 创建客户端
    
    def 发送请求(self):
        # 创建请求
        # 发送请求并等待响应
        # 处理响应

# 3. 主函数流程
def main():
    # 初始化ROS2系统
    # 创建节点实例
    # 运行节点
    # 清理资源
```

### 关键逻辑点
1. **服务端**：创建服务端，定义请求处理函数
2. **客户端**：创建客户端，发送请求并处理响应
3. **服务类型**：使用标准服务类型（如AddTwoInts）
4. **请求-响应**：客户端发送请求，服务端返回响应

## 创建服务通信的基本要素

### 1. 服务端（Server）
提供服务的组件，处理客户端请求。

### 2. 客户端（Client）
请求服务的组件，发送请求并接收响应。

### 3. 服务类型
定义请求和响应数据结构的类型。

**示例：AddTwoInts服务类型**
```
int64 a      # 请求：第一个整数
int64 b      # 请求：第二个整数
---          # 分隔线
int64 sum    # 响应：两个整数的和
```

- 客户端发送两个整数（a、b）
- 服务端计算和并返回结果（sum）

### 4. 服务名称
服务端和客户端连接的服务标识。

### 5. 回调函数
服务端处理请求的函数。

## 代码实现

### 服务端节点
```python
#!/usr/bin/env python3
"""
服务端节点示例

这个节点演示了如何创建服务端：
1. 导入必要的ROS2库和服务类型
2. 创建服务端节点类
3. 处理客户端请求
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

# AddTwoInts 服务类型说明：
# 这是一个标准的ROS2服务类型，用于演示两个整数相加的功能
# 
# 服务类型定义（通过 ros2 interface show example_interfaces/srv/AddTwoInts 查看）：
#     int64 a      # 请求参数：第一个整数
#     int64 b      # 请求参数：第二个整数
#     ---          # 分隔线：分隔请求和响应部分
#     int64 sum    # 响应参数：两个整数的和
#
# 服务结构：
#   - 请求（Request）：包含两个整数 a 和 b
#   - 响应（Response）：包含一个整数 sum（a + b 的结果）
# 使用方式：
#   - request.a: 第一个整数
#   - request.b: 第二个整数  
#   - response.sum: 返回两个整数的和



class ServiceNode(Node):
    """
    服务端节点类
    
    这个类继承自Node基类，实现了服务端功能：
    - 创建服务端
    - 处理客户端请求
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建服务端
        """
        # 调用父类构造函数，设置节点名称为 'service_node'
        super().__init__('service_node')
        
        # 创建服务端，提供AddTwoInts服务
        # AddTwoInts: 服务类型，定义请求和响应的数据结构
        # 'add_two_ints': 服务名称，客户端通过此名称连接服务
        # self.add_two_ints_callback: 请求处理函数，处理客户端请求
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )
        
        # 输出初始化日志
        self.get_logger().info('服务端节点已启动，等待客户端请求')
    
    def add_two_ints_callback(self, request, response):
        """
        请求处理回调函数
        
        这个函数会在收到客户端请求时被调用，用于：
        - 处理客户端请求
        - 计算响应结果
        - 返回响应
        
        参数说明：
        - request: AddTwoInts.Request 类型，包含客户端发送的两个整数
        - response: AddTwoInts.Response 类型，用于返回计算结果
        """
        # 获取请求中的两个整数
        # request.a: 第一个整数（来自客户端请求）
        # request.b: 第二个整数（来自客户端请求）
        a = request.a
        b = request.b
        
        # 计算两个整数的和，并设置到响应中
        # response.sum: 返回给客户端的计算结果
        response.sum = a + b
        
        # 输出处理日志
        self.get_logger().info(f'收到请求: {a} + {b} = {response.sum}')
        
        # 返回响应
        return response


def main(args=None):
    """
    主函数 - 服务端节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建服务端节点实例
    3. 运行节点
    4. 清理资源
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    
    # 创建ServiceNode实例
    service_node = ServiceNode()
    
    # 运行节点，开始处理回调
    rclpy.spin(service_node)
    
    # 关闭ROS2客户端库
    rclpy.shutdown()


if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行这个Python文件时，会执行main()函数
    """
    main()
```

### 客户端节点
```python
#!/usr/bin/env python3
"""
客户端节点示例

这个节点演示了如何创建客户端：
1. 导入必要的ROS2库和服务类型
2. 创建客户端节点类
3. 发送请求并处理响应
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ClientNode(Node):
    """
    客户端节点类
    
    这个类继承自Node基类，实现了客户端功能：
    - 创建客户端
    - 发送请求并处理响应
    """
    
    def __init__(self):
        """
        节点初始化函数
        
        在这里我们：
        1. 调用父类构造函数，设置节点名称
        2. 创建客户端
        3. 创建定时器，定期发送请求
        """
        # 调用父类构造函数，设置节点名称为 'client_node'
        super().__init__('client_node')
        
        # 创建客户端，连接到'add_two_ints'服务
        # AddTwoInts: 服务类型，与服务端保持一致
        # 'add_two_ints': 服务名称，必须与服务端相同
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # 创建定时器，每2秒执行一次timer_callback函数
        self.timer = self.create_timer(2.0, self.timer_callback)
        
        # 输出初始化日志
        self.get_logger().info('客户端节点已启动')
    
    def timer_callback(self):
        """
        定时器回调函数
        
        这个函数会被定时器定期调用，用于：
        - 发送服务请求
        - 处理服务响应
        """
        # 等待服务端可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务端启动...')
        
        # 创建请求
        # AddTwoInts.Request(): 创建AddTwoInts服务类型的请求对象
        # request.a: 设置第一个整数
        # request.b: 设置第二个整数
        request = AddTwoInts.Request()
        request.a = 10
        request.b = 20
        
        # 发送请求并等待响应
        future = self.client.call_async(request)
        
        # 处理响应
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """
        响应处理回调函数
        
        这个函数会在收到服务端响应时被调用，用于：
        - 处理服务端响应
        - 输出响应结果
        """
        try:
            # 获取响应结果
            # future.result(): 获取异步调用的结果，返回AddTwoInts.Response对象
            response = future.result()
            
            # 输出响应日志
            # response.sum: 获取服务端返回的计算结果
            self.get_logger().info(f'收到响应: {response.sum}')
            
        except Exception as e:
            # 处理异常
            self.get_logger().error(f'服务调用失败: {e}')


def main(args=None):
    """
    主函数 - 客户端节点的入口点
    
    这个函数负责：
    1. 初始化ROS2客户端库
    2. 创建客户端节点实例
    3. 运行节点
    4. 清理资源
    """
    # 初始化ROS2客户端库
    rclpy.init(args=args)
    
    # 创建ClientNode实例
    client_node = ClientNode()
    
    # 运行节点，开始处理回调
    rclpy.spin(client_node)
    
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

### 服务端实现
```python
# 创建服务端
self.srv = self.create_service(
    AddTwoInts, 'add_two_ints', self.add_two_ints_callback
)

# 请求处理函数
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    return response
```
**对应伪代码**：`# 创建服务端` 和 `# 处理客户端请求`
- 创建AddTwoInts类型的服务端（AddTwoInts定义请求包含a、b两个整数，响应包含sum一个整数）
- 设置服务名称为'add_two_ints'
- 定义请求处理回调函数（从request获取a、b，计算结果存入response.sum）

### 客户端实现
```python
# 创建客户端
self.client = self.create_client(AddTwoInts, 'add_two_ints')

# 发送请求
request = AddTwoInts.Request()
request.a = 10
request.b = 20
future = self.client.call_async(request)
```
**对应伪代码**：`# 创建客户端` 和 `# 发送请求并等待响应`
- 创建AddTwoInts类型的客户端（与服务端使用相同的服务类型）
- 连接到'add_two_ints'服务（服务名称必须与服务端一致）
- 创建请求并异步发送（AddTwoInts.Request()创建请求对象，设置a、b值）

## 服务通信的生命周期

### 1. 初始化阶段
- 导入服务类型
- 创建服务端或客户端
- 设置服务名称

### 2. 运行阶段
- 服务端：等待并处理客户端请求
- 客户端：发送请求并处理响应

### 3. 清理阶段
- 销毁节点
- 关闭ROS2系统

## 关键概念

### 服务名称
服务端和客户端通过相同的服务名称进行连接。

### 服务类型
定义请求和响应数据结构的类型，如AddTwoInts：

```
int64 a      # 请求参数：第一个整数
int64 b      # 请求参数：第二个整数
---          # 分隔线
int64 sum    # 响应参数：两个整数的和
```

服务类型通过分隔线（---）将请求和响应部分分开，客户端发送请求数据，服务端返回响应数据。

### 异步调用
客户端使用异步方式发送请求，不阻塞节点运行。

### 回调函数
服务端处理请求的函数，在收到请求时自动调用。

## 话题 vs 服务

| 特性 | 话题（Topic） | 服务（Service） |
|------|---------------|-----------------|
| **通信模式** | 发布-订阅 | 请求-响应 |
| **同步性** | 异步 | 同步 |
| **数据流向** | 单向 | 双向 |
| **适用场景** | 持续数据流 | 功能调用 |
| **连接关系** | 一对多 | 一对一 |
