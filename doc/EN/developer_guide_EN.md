# Developer Guide

This guide is intended for developers who want to use Fourier Aurora SDK for their own applications on Fourier robots. It provides information on how you can use the SDK based on your project's needs.

## Using the Controllers Provided in Packages

Aurora provides a set of controllers that can be used to control the robot's motion. Some universal controllers are provided in the `fourier-aurora` package, while others are provided in the expansion packages according to the robot configuration, such as `fourier-aurora-gr2` for GR-2 robot. 

Aurora runs a FSM state machine and each state corresponds to certain tasks(controllers). You can switch between these states using the joystick or by sending commands through the DDS interface. Every task has its own inputs and outputs. You can use **joystick** or **DDS** to interact with the tasks.

Descriptions of the states and controllers can be found in [Controller Reference](./robot_controller_reference_EN.md).


### Joystick Control

Joystick provides limited inputs to Aurora, including state change, velocity control, stand pose adjustment. For detailed information on joystick control, please refer to the [Joystick Tutorial](../../doc/EN/joystick_tutorial_EN.md).

### DDS Control

DDS (Data Distribution Service) is a communication protocol that allows Aurora to communicate with other programs or devices. DDS provides a flexible and efficient way to change Aurora's states. And it can be used to control Aurora's motion, as well as to get information from Aurora, such as its current state, joint angles, and sensor readings.

#### Python DDS Interface

Aurora provides a Python DDS client that allows you to send and receive data from Aurora using DDS. Please refer to the `python` directory for more information.

#### C++ DDS Interface

For C++ developers, there is no C++ DDS client provided in Aurora now. However, you can use the C++ fourierdds interface provided to write publishers and subscribers to interact with Aurora. Please refer to the `example/gr2/dds_comm_task ` directory for more information.

## Writing Your Own Controllers in Aurora

To write your own controllers in Aurora, please refer to the [Task Tutorial](../../doc/EN/task_tutorial_EN.md) for more information.