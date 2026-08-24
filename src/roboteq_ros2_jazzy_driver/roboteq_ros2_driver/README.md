# roboteq_ros2_driver


ROS2 driver for the Roboteq SDC21xx, HDC24xx family of motor controllers in a differential-drive configuration.
Initially developed for SDC21xx and HDC24xx, but could work with other roboteq dual-channel motor drivers.

Subscribes to cmd_vel, publishes to odom


Does not require any MicroBasic script to operate.

## Usage

Clone to src directory of ros2 workspace, then `colcon build` 

Requires serial package, which is not available as deb in ROS2. If not already installed, install ros2 branch of serial:

Get the code:
    
    git clone -b ros2 https://github.com/SunnyApp-Robotics/serial.git

Build:

    make

Install:

    make install
    
    
Sample launch files in roboteq_ros2_driver/launch, or run `ros2 run roboteq_ros2_driver roboteq_ros2_driver`

## Motor Power Connections

This driver assumes right motor is connected to channel 1 (M1) of motor controller, and left motor is connected to channel 2 (M2). It also assumes a positive speed command will result in forward motion of each motor. Best to test motor directions using the roboteq utility software.

## Per-robot direction profile

Motor wiring and encoder polarity can be selected without changing the shared
driver code. The four direction parameters accept `1.0` or `-1.0`:

| Parameter | Purpose | Shared default |
| --- | --- | ---: |
| `motor_sign_r` | Right motor (M1) command direction | `1.0` |
| `motor_sign_l` | Left motor (M2) command direction | `-1.0` |
| `encoder_sign_r` | Right encoder odometry direction | `-1.0` |
| `encoder_sign_l` | Left encoder odometry direction | `1.0` |

The workspace launcher reads an optional machine-local profile from
`~/.config/sirius/roboteq_profile.sh`. For example, a robot whose two motors and
two encoders both use the negative direction can use:

```bash
export SIRIUS_MOTOR_SIGN_R=-1.0
export SIRIUS_MOTOR_SIGN_L=-1.0
export SIRIUS_ENCODER_SIGN_R=-1.0
export SIRIUS_ENCODER_SIGN_L=-1.0
```

Robots without this local profile keep the shared defaults above.


## TODO

- [X] Initial ROS2 release with motor commands and odometry stream
- [ ] Implement transform broadcasting with tf2
- [ ] Add roboteq/voltage, roboteq/current, roboteq/energy, and roboteq/temperature publishers
- [ ] Make topic names and frames configuration parameters configurable at runtime.
- [ ] Make robot configuration parameters configurable at runtime.
- [ ] Make motor controller device configuration parameters configurable at runtime.
- [ ] Make miscellaneous motor controller configuration parameters configurable at runtime.
- [ ] Implement dynamically enabled self-test mode to verify correct motor power and encoder connections and configuration.

### Note: I do not have access to Roboteq hardware anymore - feel free to contribute!
[original work for ROS1](https://github.com/ecostech/roboteq_diff_driver)
## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)
* **Chase Devitt** - *ROS2 Migration*
