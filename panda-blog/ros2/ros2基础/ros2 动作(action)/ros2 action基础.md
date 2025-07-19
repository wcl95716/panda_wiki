# ROS2 动作基础

## 什么是动作（Action）？

ROS2动作（Action）是ROS2中节点间通信的第三种方式，专门用于处理**长时间执行的任务**。与服务的"请求-响应"模式不同，动作采用"目标-反馈-结果"模式，允许客户端发送任务目标后，持续监控任务的执行进度。

### 动作的核心特点
- **长时间任务**：适合执行需要较长时间的任务（如导航、路径规划）
- **进度反馈**：服务端在执行过程中可以持续发送进度信息
- **异步执行**：客户端发送目标后可以继续执行其他任务
- **可取消性**：客户端可以随时取消正在执行的动作
- **状态跟踪**：可以实时跟踪动作的执行状态

## 为什么需要 Action？

### 服务的局限性
使用服务处理长时间任务时的问题：

```text
客户端：请导航到坐标(10, 20)
服务端：[等待...等待...等待...]
客户端：[继续等待，无法知道进度]
服务端：导航完成！
```

**问题**：
- 客户端必须一直等待，无法知道进度
- 长时间任务会阻塞客户端
- 无法中途取消任务
- 无法获得实时状态

### Action 的优势
使用 Action 处理同样的任务：

```text
客户端：请导航到坐标(10, 20)
服务端：目标已接受，开始导航
服务端：反馈：已移动 20%，距离目标 8米
服务端：反馈：已移动 50%，距离目标 5米
客户端：收到反馈，知道进度
服务端：反馈：已移动 80%，距离目标 2米
服务端：导航完成！最终位置：(10.1, 19.9)
```

**优势**：
- 客户端可以实时了解任务进度
- 客户端不会被阻塞，可以处理其他任务
- 可以中途取消任务
- 获得详细的执行状态信息

## Action vs 传统异步任务模式

### 传统异步任务模式
```text
1. 客户端发送请求 → 服务端
2. 服务端返回任务ID → 客户端
3. 客户端定期查询任务ID状态 → 服务端
4. 服务端返回任务状态和进度 → 客户端
5. 任务完成后，客户端获取最终结果 → 服务端
```

### ROS2 Action 模式
```text
1. 客户端发送目标(Goal) → 服务端
2. 服务端返回 Goal ID → 客户端
3. 服务端主动推送状态和进度 → 客户端
4. 客户端接收实时反馈 → 服务端
5. 任务完成后，服务端推送最终结果 → 客户端
```

### 关键差异

| 特性 | 传统轮询模式 | ROS2 Action 模式 |
|------|-------------|------------------|
| **查询方式** | 客户端主动轮询 | 服务端主动推送 |
| **实时性** | 取决于轮询频率 | 立即推送 |
| **网络效率** | 大量无效请求 | 按需推送 |
| **取消支持** | 需要额外接口 | 内置支持 |
| **多客户端** | 需要额外管理 | 自动管理 |
| **状态管理** | 客户端负责 | 服务端负责 |
| **错误处理** | 需要额外处理 | 内置处理 |

### 代码对比

#### 传统异步任务模式
```python
# 1. 发送任务请求
response = requests.post('/api/tasks', json={'target': 'navigate_to_10_20'})
task_id = response.json()['task_id']

# 2. 轮询任务状态
while True:
    status_response = requests.get(f'/api/tasks/{task_id}')
    status = status_response.json()
    
    if status['state'] == 'completed':
        result = status['result']
        break
    elif status['state'] == 'failed':
        error = status['error']
        break
    
    time.sleep(1)  # 等待1秒后再次查询
```

#### ROS2 Action 模式
```python
# 1. 发送目标
goal_msg = Navigation.Goal()
goal_msg.target_x = 10
goal_msg.target_y = 20

# 2. 设置回调，自动接收状态更新
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg, feedback_callback=self.feedback_callback
)

# 3. 回调函数自动处理状态更新
def feedback_callback(self, feedback_msg):
    # 自动接收进度反馈，无需轮询
    progress = feedback_msg.feedback.progress
    print(f"导航进度: {progress}%")
```

### 优势总结

Action 本质上就是一个**智能化的异步任务系统**：

1. **请求阶段**：客户端发送目标，获得 Goal ID
2. **监控阶段**：服务端主动推送状态和进度（而不是客户端轮询）
3. **完成阶段**：服务端推送最终结果

**Action 的优势**：
- **更高效**：避免无效的轮询请求
- **更实时**：状态变化立即推送
- **更智能**：内置取消、错误处理等机制
- **更简单**：客户端不需要管理轮询逻辑

所以 Action 可以看作是传统异步任务模式的**升级版**，提供了更好的用户体验和系统效率。

## Action 的组成部分

### 1. 目标（Goal）
客户端发送给服务端的任务目标。

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
服务端完成任务后返回的最终结果。

**示例**：
- 导航结果：最终位置、总耗时、路径长度
- 路径规划结果：完整路径、路径质量评分
- 机械臂结果：最终位置、执行状态

### 4. 取消（Cancel）
客户端可以发送取消请求来停止动作执行。

### 5. 状态（Status）
服务端发送的动作执行状态信息，包括当前状态和进度。

## Action 执行过程详解

### 核心概念

#### goal_handle：服务端的核心对象
`goal_handle` 是 Action 服务端中最关键的对象，负责管理整个动作的生命周期：

**核心作用**：
1. **状态管理**：跟踪动作执行状态（ACCEPTED → EXECUTING → SUCCEEDED/CANCELED/ABORTED）
2. **进度反馈**：向客户端发送实时进度信息
3. **取消检查**：检测客户端是否发送了取消请求
4. **结果返回**：在任务完成时返回最终结果

**关键方法**：
| 方法 | 作用 | 使用场景 |
|------|------|----------|
| `is_cancel_requested` | 检查是否被取消 | 在循环中检查取消状态 |
| `publish_feedback()` | 发送进度反馈 | 定期发送执行进度 |
| `succeed()` | 标记成功完成 | 任务正常完成时 |
| `abort()` | 标记执行失败 | 任务执行出错时 |
| `canceled()` | 标记被取消 | 收到取消请求时 |

### Action 执行流程

#### 1. 客户端发送端流程
```python
# 1. 创建动作客户端
self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

# 2. 发送目标（异步）
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg, feedback_callback=self.feedback_callback
)

# 3. 设置回调函数链
self._send_goal_future.add_done_callback(self.goal_response_callback)
```

**客户端回调函数**：
- `feedback_callback`：实时处理进度反馈
- `goal_response_callback`：处理目标是否被接受
- `get_result_callback`：处理最终结果

#### 2. 服务端处理端流程
```python
# 1. 创建动作服务端
self._action_server = ActionServer(
    self, Fibonacci, 'fibonacci', self.execute_callback
)

# 2. 处理客户端目标
def execute_callback(self, goal_handle):
    # 获取目标参数
    # 执行任务循环
    # 发送反馈
    # 返回结果
```

### Action 底层通信机制

Action 使用 5 个话题实现完整通信：

1. **Goal Topic**：客户端 → 服务端（发送目标）
2. **Feedback Topic**：服务端 → 客户端（发送进度反馈）
3. **Result Topic**：服务端 → 客户端（发送最终结果）
4. **Cancel Topic**：客户端 → 服务端（发送取消请求）
5. **Status Topic**：服务端 → 客户端（发送状态更新）

### Action ID 和客户端识别机制

#### 为什么需要 Action ID？

当多个客户端同时向同一个服务端发送 Action 请求时，系统需要能够区分不同的请求：

```text
客户端A：发送导航目标 (x=10, y=20)
客户端B：发送导航目标 (x=30, y=40)
客户端C：发送导航目标 (x=50, y=60)

服务端需要知道：
- 哪个反馈应该发送给哪个客户端？
- 哪个取消请求对应哪个正在执行的任务？
- 哪个结果应该返回给哪个客户端？
```

#### Action ID 的工作原理

每个 Action 请求都有一个唯一的 **Goal ID**，用于跟踪整个 Action 的生命周期：

```python
# 客户端发送目标时，系统自动生成唯一的 Goal ID
goal_msg = Fibonacci.Goal()
goal_msg.order = 10

# 发送目标，系统自动分配 Goal ID
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg, feedback_callback=self.feedback_callback
)

# 在 goal_response_callback 中获取 Goal ID
def goal_response_callback(self, future):
    goal_handle = future.result()
    if goal_handle.accepted:
        # 保存 Goal ID 用于后续操作
        self._goal_id = goal_handle.goal_id
        self.get_logger().info(f'目标已接受，Goal ID: {self._goal_id}')
```

#### 服务端如何使用 Goal ID

服务端通过 `goal_handle` 自动管理每个 Action 请求：

```python
def execute_callback(self, goal_handle):
    """每个 goal_handle 对应一个唯一的 Action 请求"""
    
    # 获取这个特定请求的 Goal ID
    goal_id = goal_handle.goal_id
    self.get_logger().info(f'开始处理 Goal ID: {goal_id}')
    
    # 获取这个特定请求的目标参数
    order = goal_handle.request.order
    
    # 执行任务，发送反馈给对应的客户端
    for i in range(2, order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()
        
        # 发送反馈给特定的客户端
        feedback = Fibonacci.Feedback()
        feedback.sequence = sequence
        goal_handle.publish_feedback(feedback)  # 自动发送给正确的客户端
    
    # 返回结果给特定的客户端
    result = Fibonacci.Result()
    result.sequence = sequence
    goal_handle.succeed()  # 自动发送给正确的客户端
    return result
```

#### 多客户端场景示例

```python
# 场景：3个客户端同时发送导航请求

# 客户端A
client_a = ActionClient(node, Navigation, 'navigation')
goal_a = Navigation.Goal()
goal_a.target_x = 10
goal_a.target_y = 20
# 系统自动分配 Goal ID: "goal_001"

# 客户端B  
client_b = ActionClient(node, Navigation, 'navigation')
goal_b = Navigation.Goal()
goal_b.target_x = 30
goal_b.target_y = 40
# 系统自动分配 Goal ID: "goal_002"

# 客户端C
client_c = ActionClient(node, Navigation, 'navigation')
goal_c = Navigation.Goal()
goal_c.target_x = 50
goal_c.target_y = 60
# 系统自动分配 Goal ID: "goal_003"

# 服务端同时处理3个请求
# - goal_handle_001 处理客户端A的请求
# - goal_handle_002 处理客户端B的请求  
# - goal_handle_003 处理客户端C的请求
# 每个 goal_handle 自动将反馈和结果发送给对应的客户端
```

#### 取消特定 Action

客户端可以取消特定的 Action 请求：

```python
def cancel_specific_goal(self):
    """取消特定的 Action 请求"""
    if hasattr(self, '_goal_handle'):
        # 取消特定的 Goal ID
        self._goal_handle.cancel_goal_async()
        self.get_logger().info(f'已发送取消请求，Goal ID: {self._goal_handle.goal_id}')
```

#### 底层实现

在底层，Action 系统使用以下机制确保正确的客户端识别：

1. **Goal ID**：每个 Action 请求的唯一标识符
2. **客户端连接**：每个 `goal_handle` 维护与特定客户端的连接
3. **消息路由**：系统根据 Goal ID 自动路由反馈和结果消息
4. **状态跟踪**：服务端跟踪每个 Goal ID 的执行状态

这种设计确保了：
- 多个客户端可以同时使用同一个 Action 服务
- 每个客户端只收到自己请求的反馈和结果
- 客户端可以独立取消自己的请求
- 服务端可以同时处理多个 Action 请求

### Action 执行生命周期

```text
客户端发送目标
    ↓
服务端接受目标 (ACCEPTED)
    ↓
服务端开始执行 (EXECUTING)
    ↓
服务端发送反馈 (EXECUTING + 反馈)
    ↓
服务端完成任务 (SUCCEEDED/ABORTED/CANCELED)
    ↓
服务端返回结果
```

### Action 状态详解

Action 有 6 个主要状态：

1. **ACCEPTED**：目标被服务端接受，准备开始执行
2. **EXECUTING**：动作正在执行中，服务端发送反馈
3. **CANCELING**：动作正在被取消（如果客户端发送取消请求）
4. **CANCELED**：动作已被取消
5. **SUCCEEDED**：动作执行成功，返回结果
6. **ABORTED**：动作执行失败

**状态转换示例**：
```text
正常完成：ACCEPTED → EXECUTING → SUCCEEDED
被取消：  ACCEPTED → EXECUTING → CANCELING → CANCELED
执行失败：ACCEPTED → EXECUTING → ABORTED
```

## 代码实现

### 动作服务端
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
        """处理客户端目标 - goal_handle 是核心对象"""
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

### 动作客户端
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
        # 创建目标消息
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        # 异步发送目标
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
        
        # 保存 goal_handle 和 Goal ID 用于后续操作
        self._goal_handle = goal_handle
        self._goal_id = goal_handle.goal_id
        self.get_logger().info(f'目标已接受，Goal ID: {self._goal_id}')
        
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
```text
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

## 高级功能

### 取消机制

客户端可以随时取消正在执行的动作：

```python
# 客户端取消动作
def cancel_goal(self):
    if hasattr(self, '_goal_handle'):
        self._goal_handle.cancel_goal_async()
        self.get_logger().info(f'已发送取消请求，Goal ID: {self._goal_id}')

# 服务端检查取消
def execute_callback(self, goal_handle):
    goal_id = goal_handle.goal_id
    while not task_completed:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info(f'Goal ID {goal_id} 已被取消')
            return
        # 继续执行任务
```

### 目标拒绝机制

服务端可以拒绝客户端的目标请求：

```python
def execute_callback(self, goal_handle):
    # 检查目标是否合理
    if goal_handle.request.order > 100:
        goal_handle.abort()  # 拒绝目标
        return Fibonacci.Result()
    
    # 正常处理目标
    # ...
```

### 错误处理

服务端可以处理执行过程中的错误：

```python
def execute_callback(self, goal_handle):
    try:
        # 执行任务
        result = self.perform_task(goal_handle.request)
        goal_handle.succeed()
        return result
    except Exception as e:
        # 处理错误
        goal_handle.abort()
        self.get_logger().error(f'任务执行失败: {e}')
        return Fibonacci.Result()
```

### 超时处理

客户端可以设置超时时间：

```python
# 客户端设置超时
def send_goal_with_timeout(self):
    goal_msg = Fibonacci.Goal()
    goal_msg.order = 10
    
    # 发送目标并设置超时
    self._send_goal_future = self._action_client.send_goal_async(
        goal_msg, feedback_callback=self.feedback_callback
    )
    
    # 等待目标响应，超时时间 5 秒
    if not self._send_goal_future.done():
        self.get_logger().warn('目标发送超时')
        return
```

### 多客户端取消场景

```python
# 场景：多个客户端同时取消各自的 Action

# 客户端A 取消自己的导航请求
client_a.cancel_goal()  # 取消 Goal ID: "goal_001"

# 客户端B 取消自己的导航请求  
client_b.cancel_goal()  # 取消 Goal ID: "goal_002"

# 客户端C 的导航请求继续执行
# Goal ID: "goal_003" 继续执行

# 服务端分别处理每个取消请求
# - goal_handle_001 收到取消请求，停止执行
# - goal_handle_002 收到取消请求，停止执行
# - goal_handle_003 继续正常执行
```

## 实际应用场景

### 机器人导航
```python
# 导航 Action 示例
class NavigationAction:
    def __init__(self):
        self._action_server = ActionServer(
            self, Navigation, 'navigation', self.navigate_callback
        )
    
    def navigate_callback(self, goal_handle):
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y
        
        # 开始导航
        while not self.reached_target():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return Navigation.Result()
            
            # 发送导航反馈
            feedback = Navigation.Feedback()
            feedback.current_x = self.current_x
            feedback.current_y = self.current_y
            feedback.distance_to_goal = self.calculate_distance()
            goal_handle.publish_feedback(feedback)
            
            # 执行导航步骤
            self.move_towards_target()
        
        # 导航完成
        result = Navigation.Result()
        result.final_x = self.current_x
        result.final_y = self.current_y
        goal_handle.succeed()
        return result
```

### 机械臂控制
```python
# 机械臂 Action 示例
class ArmAction:
    def __init__(self):
        self._action_server = ActionServer(
            self, ArmControl, 'arm_control', self.arm_callback
        )
    
    def arm_callback(self, goal_handle):
        target_position = goal_handle.request.target_position
        
        # 控制机械臂移动
        for step in self.plan_movement(target_position):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ArmControl.Result()
            
            # 发送关节状态反馈
            feedback = ArmControl.Feedback()
            feedback.joint_angles = self.get_joint_angles()
            feedback.movement_progress = step.progress
            goal_handle.publish_feedback(feedback)
            
            # 执行移动步骤
            self.execute_movement_step(step)
        
        # 移动完成
        result = ArmControl.Result()
        result.final_position = self.get_current_position()
        goal_handle.succeed()
        return result
```

### 图像处理
```python
# 图像处理 Action 示例
class ImageProcessingAction:
    def __init__(self):
        self._action_server = ActionServer(
            self, ImageProcess, 'image_process', self.process_callback
        )
    
    def process_callback(self, goal_handle):
        image = goal_handle.request.image
        
        # 分步骤处理图像
        for step in self.image_processing_pipeline():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return ImageProcess.Result()
            
            # 发送处理进度反馈
            feedback = ImageProcess.Feedback()
            feedback.processing_step = step.name
            feedback.progress_percentage = step.progress
            goal_handle.publish_feedback(feedback)
            
            # 执行处理步骤
            image = step.process(image)
        
        # 处理完成
        result = ImageProcess.Result()
        result.processed_image = image
        goal_handle.succeed()
        return result
```

## 总结

Action 是 ROS2 中处理长时间任务的最佳选择，具有以下优势：

1. **提供实时反馈**：让用户了解任务进度
2. **支持异步执行**：不阻塞客户端
3. **允许中途取消**：提供更大的控制权
4. **状态跟踪**：实时了解任务状态

### 核心要点

- **goal_handle** 是服务端的核心对象，负责管理动作的整个生命周期
- **Goal ID** 是每个 Action 请求的唯一标识符，用于区分不同的客户端请求
- Action 使用 5 个话题实现完整通信：Goal、Feedback、Result、Cancel、Status
- 客户端通过回调函数链处理反馈和结果
- 服务端通过 goal_handle 发送反馈和返回结果
- 多个客户端可以同时使用同一个 Action 服务，系统自动管理各自的请求
- Action 有 6 个状态：ACCEPTED、EXECUTING、CANCELING、CANCELED、SUCCEEDED、ABORTED
- 支持目标拒绝、错误处理、超时处理等高级功能

### 与传统异步任务模式的对比

- **查询方式**：Action 是服务端主动推送，传统模式是客户端轮询
- **实时性**：Action 立即推送，传统模式取决于轮询频率
- **网络效率**：Action 按需推送，传统模式有大量无效请求
- **功能完整性**：Action 内置取消、错误处理，传统模式需要额外实现
- **开发复杂度**：Action 更简单，传统模式需要管理轮询逻辑

### 适用场景

Action 特别适合以下场景：
- **机器人导航**：需要实时反馈位置和进度
- **机械臂控制**：需要监控关节状态和运动进度
- **图像处理**：需要了解处理进度和中间结果
- **路径规划**：需要了解规划进度和优化状态
- **语音识别**：需要实时显示识别进度
- **机器学习训练**：需要监控训练进度和损失值
- **文件传输**：需要显示传输进度和速度
- **数据备份**：需要显示备份进度和剩余时间
