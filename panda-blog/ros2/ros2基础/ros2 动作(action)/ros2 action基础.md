# ROS2 动作基础

## 什么是动作（Action）？

ROS2动作（Action）是ROS2中节点间通信的第三种方式，专门用于处理**长时间执行的任务**。与服务的"请求-响应"模式不同，动作采用"目标-反馈-结果"模式，允许客户端发送任务目标后，持续监控任务的执行进度。

### 动作的核心特点
- **长时间任务**：适合执行需要较长时间的任务（如导航、路径规划）
- **进度反馈**：服务端在执行过程中可以持续发送进度信息
- **异步执行**：客户端发送目标后可以继续执行其他任务
- **可取消性**：客户端可以随时取消正在执行的动作
- **状态跟踪**：可以实时跟踪动作的执行状态

## 动作 vs 服务：理解差异

### 服务（Service）的局限性
想象一下，如果你使用服务来请求机器人导航到某个位置：

```
客户端：请导航到坐标(10, 20)
服务端：[等待...等待...等待...]
客户端：[继续等待，无法知道进度]
服务端：导航完成！
```

**问题**：
- 客户端必须一直等待，无法知道导航进度
- 如果导航时间很长，客户端会被阻塞
- 无法中途取消导航任务
- 无法获得实时的执行状态

### 动作（Action）的优势
使用动作来处理同样的导航任务：

```
客户端：请导航到坐标(10, 20)
服务端：目标已接受，开始导航
服务端：反馈：已移动 20%，距离目标 8米
服务端：反馈：已移动 50%，距离目标 5米
服务端：反馈：已移动 80%，距离目标 2米
客户端：收到反馈，知道进度
服务端：反馈：已移动 95%，距离目标 0.5米
服务端：导航完成！最终位置：(10.1, 19.9)
```

**优势**：
- 客户端可以实时了解任务进度
- 客户端不会被阻塞，可以处理其他任务
- 可以中途取消任务
- 获得详细的执行状态信息

## 动作通信的组成部分

### 1. 目标（Goal）
客户端发送给服务端的任务目标，类似于服务的请求。

**示例**：
- 导航目标：目标坐标、速度限制
- 路径规划：起点、终点、路径类型
- 机械臂控制：目标位置、运动模式

### 2. 反馈（Feedback）
服务端在执行过程中持续发送的进度信息。

**示例**：
- 导航反馈：当前位置、距离目标距离、预计剩余时间
- 路径规划反馈：已规划路径长度、当前规划进度
- 机械臂反馈：当前关节角度、运动速度

### 3. 结果（Result）
服务端完成任务后返回的最终结果，类似于服务的响应。

**示例**：
- 导航结果：最终位置、总耗时、路径长度
- 路径规划结果：完整路径、路径质量评分
- 机械臂结果：最终位置、执行状态

### 4. 取消（Cancel）
客户端可以发送取消请求来停止动作执行。

## Action的核心价值

### 场景对比分析

| 场景 | 用Topic | 用Service | 用Action | 说明 |
|------|---------|-----------|----------|------|
| **机器人导航** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要实时反馈位置和进度 |
| **机械臂抓取** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要监控关节状态和抓取进度 |
| **图像处理** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要了解处理进度和中间结果 |
| **文件下载** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要显示下载进度和速度 |
| **路径规划** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要了解规划进度和优化状态 |
| **语音识别** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要实时显示识别进度 |
| **数据备份** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要显示备份进度和剩余时间 |
| **机器学习训练** | ❌ 无法控制 | ❌ 不知道进度 | ✅ 完美！ | 需要监控训练进度和损失值 |

### 为什么Action完美适合长时间任务？

- **实时反馈**：可以持续发送进度信息
- **可控制**：支持中途取消和暂停
- **非阻塞**：客户端发送目标后可以处理其他任务
- **状态跟踪**：可以实时了解任务执行状态

### 动作与话题、服务的对比

| 特性 | 话题（Topic） | 服务（Service） | 动作（Action） |
|------|---------------|-----------------|----------------|
| **通信模式** | 发布-订阅 | 请求-响应 | 目标-反馈-结果 |
| **同步性** | 异步 | 同步 | 异步 |
| **数据流向** | 单向 | 双向 | 双向 |
| **反馈机制** | 无 | 无 | 有 |
| **可取消性** | 无 | 无 | 有 |
| **适用场景** | 持续数据流 | 快速功能调用 | 长时间任务 |
| **阻塞性** | 不阻塞 | 阻塞 | 不阻塞 |
| **状态跟踪** | 无 | 无 | 有 |
| **进度监控** | 无 | 无 | 有 |
| **用户体验** | 一般 | 差（长时间等待） | 优秀 |

## 动作状态管理

### 动作的生命周期状态

1. **ACCEPTED**：目标被服务端接受，准备开始执行
2. **EXECUTING**：动作正在执行中，服务端发送反馈
3. **CANCELING**：动作正在被取消（如果客户端发送取消请求）
4. **CANCELED**：动作已被取消
5. **SUCCEEDED**：动作执行成功，返回结果
6. **ABORTED**：动作执行失败

### 状态转换示例
```
客户端发送目标 → ACCEPTED
服务端开始执行 → EXECUTING
服务端发送反馈 → EXECUTING（继续）
服务端完成任务 → SUCCEEDED
```

或者：
```
客户端发送目标 → ACCEPTED
服务端开始执行 → EXECUTING
客户端发送取消 → CANCELING
服务端停止执行 → CANCELED
```

## 实际应用示例

### 智能家居场景
**使用服务（不合适）**：
```
用户：请打扫房间
系统：[等待...等待...等待...]
用户：不知道进度，无法取消
系统：打扫完成！
```

**使用动作（合适）**：
```
用户：请打扫房间
系统：开始打扫，预计需要30分钟
系统：反馈：正在清扫客厅，进度 20%
系统：反馈：正在清扫卧室，进度 50%
用户：收到反馈，知道进度
系统：反馈：正在清扫厨房，进度 80%
系统：打扫完成！总耗时 28分钟
```

### 自动驾驶场景
**导航任务**：
```
目标：从当前位置导航到目的地
反馈：当前速度 30km/h，距离目的地 5km，预计到达时间 10分钟
反馈：当前速度 35km/h，距离目的地 3km，预计到达时间 8分钟
反馈：当前速度 25km/h，距离目的地 1km，预计到达时间 4分钟
结果：导航完成，总耗时 15分钟，平均速度 32km/h
```

## 代码实现示例

### 伪代码逻辑

```python
# 动作服务端逻辑
class ActionServer:
    def __init__(self):
        # 创建动作服务端
        # 设置动作名称和类型
    
    def handle_goal(self, goal):
        # 接收客户端目标
        # 开始执行任务
        # 循环执行：
        #   - 检查是否被取消
        #   - 执行一步任务
        #   - 发送进度反馈
        #   - 检查是否完成
        # 返回最终结果

# 动作客户端逻辑
class ActionClient:
    def __init__(self):
        # 创建动作客户端
        # 连接到动作服务端
    
    def send_goal(self):
        # 创建目标
        # 发送目标
        # 设置反馈回调
        # 设置结果回调
    
    def feedback_callback(self, feedback):
        # 处理进度反馈
        # 显示当前进度
    
    def result_callback(self, result):
        # 处理最终结果
        # 显示完成状态
```

### 简单代码实现

#### 动作服务端
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionServer(Node):
    def __init__(self):
        super().__init__('simple_action_server')
        
        # 创建动作服务端
        self._action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.execute_callback
        )
        self.get_logger().info('动作服务端已启动')
    
    def execute_callback(self, goal_handle):
        """处理客户端目标"""
        # 获取目标参数
        order = goal_handle.request.order
        
        # 初始化斐波那契数列
        sequence = [0, 1]
        
        # 计算斐波那契数列
        for i in range(2, order):
            # 检查是否被取消
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Fibonacci.Result()
            
            # 计算下一项
            next_num = sequence[i-1] + sequence[i-2]
            sequence.append(next_num)
            
            # 发送反馈
            feedback = Fibonacci.Feedback()
            feedback.sequence = sequence
            goal_handle.publish_feedback(feedback)
            
            # 模拟计算延迟
            self.get_rate().sleep()
        
        # 返回结果
        result = Fibonacci.Result()
        result.sequence = sequence
        goal_handle.succeed()
        return result

def main():
    rclpy.init()
    node = SimpleActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 动作客户端
```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        
        # 创建动作客户端
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        
        # 等待服务端启动
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('等待动作服务端启动...')
        
        # 发送目标
        self.send_goal()
    
    def send_goal(self):
        """发送动作目标"""
        # 创建目标
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        # 发送目标
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        """处理反馈"""
        sequence = feedback_msg.feedback.sequence
        self.get_logger().info(f'收到反馈: {sequence}')
    
    def goal_response_callback(self, future):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
        
        # 获取结果
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """处理结果"""
        result = future.result().result
        self.get_logger().info(f'收到结果: {result.sequence}')

def main():
    rclpy.init()
    node = SimpleActionClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 运行示例

1. **启动动作服务端**：
```bash
python3 action_server.py
```

2. **启动动作客户端**：
```bash
python3 action_client.py
```

3. **观察输出**：
```
# 服务端输出
动作服务端已启动

# 客户端输出
等待动作服务端启动...
收到反馈: [0, 1, 1]
收到反馈: [0, 1, 1, 2]
收到反馈: [0, 1, 1, 2, 3]
...
收到结果: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34]
```

## 总结

动作是ROS2中处理长时间任务的最佳选择，它解决了服务模式在处理长时间任务时的局限性：

1. **提供实时反馈**：让用户了解任务进度
2. **支持异步执行**：不阻塞客户端
3. **允许中途取消**：提供更大的控制权
4. **状态跟踪**：实时了解任务状态

动作特别适合机器人导航、路径规划、机械臂控制等需要长时间执行且需要监控进度的任务。通过动作，我们可以构建更加用户友好和灵活的机器人系统。
