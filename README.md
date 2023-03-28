# ARCS通用接口 common_interface

为不同用户提供 ARCS 软件接口

- 为终端用户提供机器人/外部轴/IO等操作控制的 SDK 接口
- 为 aubo_control 控制器软件开发扩展功能，例如 aubo_comm/aubo_script
- 为 aubo_control 控制器软件提供基础模块，如硬件抽象层(hardware_interface)、算法、传感器等

另外，common_interface 还对 AuboControlAPI(用户SDK和扩展功能接口)单元测试提供了自动化代码生成方案(基于C++元编程)



## 1. 用户 SDK 接口

用户通过 SocketTCP/共享内存等方式与 aubo_control 的扩展功能 aubo_comm 服务端连接，发送指令给 aubo_control 控制器软件，实现对机器人的远程控制

### 1.1 JsonRPC 接口

提供了基于 SocketTCP 和 http 协议的 JsonRPC 通讯端口

### 1.2 RTDE 接口

提供了基于 SocketTCP 和 http 协议的 RTDE 接口



### 1.2 C++ SDK 接口

包括 JsonRPC 接口和 RTDE 实时数据交换接口



## 2. 控制器软件扩展

aubo_control 控制器软件内置了一个插件框架，支持在 AuboControlAPI 接口层之上对控制器软件功能进行功能扩展，例如 aubo_script 扩展了机器人脚本运行的功能，aubo_comm 是一个 socket 服务端，支持多用户连接





## 3. 控制器软件基础模块

### 3.1 算法模块

### 3.2 外部传感器

### 3.3 机器人硬件抽象层

### 3.4 外部轴硬件抽象层