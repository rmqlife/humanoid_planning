# 手柄使用教程

Aurora 默认使用手柄作为输入设备。用户可以通过手柄进行控制指令输入和系统状态切换。

本教程将指导您如何在 Aurora 系统中设置和使用手柄。

## 手柄初始设置

1. 将您的手柄连接到计算机。Aurora 使用 Linux 内置驱动 `joystick.h` 读取用户的手柄数据。要验证设备是否已正确加载到系统中，请在终端运行以下命令：

    ```bash
    ls /dev/input
    ```

    您应能看到名为 `js0` 的设备文件。Aurora 就是通过该设备文件读取手柄数据的。

2. 启动 Aurora 并检查日志输出。如果未看到以下错误信息，则说明手柄已成功连接到 Aurora：

    ```bash
    [error] joystick init fail
    [error]         task enter failed:   JoyStickTask
    ```

    如果出现上述错误信息，请尝试重新进行初始设置流程。

    **注意：**

    如果您在 Docker 容器中运行 Aurora，请务必在启动容器前连接好手柄。默认情况下，Docker 只识别容器启动时已存在的设备。如果在容器启动后插入手柄，设备文件不会自动映射到正在运行的容器内。

## Aurora 中的手柄使用

手柄在 Aurora 中主要有两个功能：切换系统状态和输入控制指令。

### 状态切换

在 Aurora 中，每个 FSM（有限状态机）状态都对应特定的肩键和按键组合。下表展示了各 FSM 状态的手柄按键映射：

| 按键组合    | 状态名称                | 状态描述                       |
|------------|------------------------|------------------------------|
| LB+RB      | Default                | 默认状态，无操作                |
| LT+RT      | SecurityProtection     | 安全保护状态，停止所有执行器      |
| LB+A       | JointStand             | 所有执行器归零位                |
| LB+B       | PdStand                | 机器人站立于平面                |
| RB+A       | UserController_A       | 预留给用户自定义任务            |
| RB+B       | UserController_B       | 预留给用户自定义任务            |
| RB+Y       | UserController_C       | 预留给用户自定义任务            |
| RB+X       | UserController_D       | 预留给用户自定义任务            |
| RB+X       | UserController_E       | 预留给用户自定义任务            |
| RB+Y       | UserController_F       | 预留给用户自定义任务            |

### 控制指令输入

#### PdStand

在 PdStand 状态下，方向键用于控制机器人姿态。`上/下` 键调节机器人上半身高度，`左/右` 键调节上半身俯仰角。

#### SimpleLocoTask

在 gr2 示例中，提供了一个简单的行走任务，默认绑定在 *UserController_A* 状态。该任务通过左右摇杆控制机器人运动。左摇杆的纵轴控制纵向（前后）运动，左摇杆的横轴控制横向（侧向）运动，右摇杆的横轴控制偏航（旋转）运动。

## 手柄布局说明

Aurora 的手柄驱动基于 Xbox 控制器布局设计。因此，推荐使用 Xbox 布局的控制器以获得最佳体验。如果您的控制器支持多种输出模式，请在配置前切换到 Xbox/PC 模式。

如需使用其他布局的控制器，请注意按键映射可能会有所不同。对于 PlayStation 控制器，`△` 键对应本教程中的 `X` 键，`□` 键对应 `Y` 键。不支持 Switch 控制器。
