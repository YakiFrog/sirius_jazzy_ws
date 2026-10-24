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
    
    
Sample launch files in roboteq_ros2_driver/launch, or run `ros2 run roboteq_ros2_driver roboteq_ros2_driver_node`

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

`max_encoder_step_revolutions` limits the accepted wheel movement from one
Roboteq encoder sample (default `0.25` revolutions). A controller counter reset
or discontinuity beyond this limit is ignored instead of being published as a
large odometry jump.


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

## Drive diagnostics

`/roboteq/diagnostics` (`roboteq_ros2_driver/msg/RoboteqDiagnostics`) supplements
`/roboteq/status`; the existing status interface is unchanged. It is recorded by
`record_rosbag`, `record_mapping`, `record_3dmap`, and `record_loc3d` on subsequent runs.

- Read-only `?FS`, `?FM`, `?P` queries run at 10 Hz independently of the original
  encoder/voltage stream. Flag labels follow the common Roboteq flag tables;
  raw values remain available for firmware-specific interpretation.
- Every MD value has validity and age fields. Unsupported, malformed, absent,
  or older-than-0.5-second replies are not healthy zero. A one-channel response
  never supplies channel 2. `md_output_*` is the raw motor-power-output query
  result, **not watts**.
- ROS FF+PI values include wheel targets, filtered feedback, errors, integrals,
  FF/P/I terms, and output before/after filtering and limits. `control_stamp`
  identifies the control sample; `control_sample_valid=false` while stopped or
  when the software closed loop is disabled. The configured D gain is not used
  by the current FF+PI implementation; no artificial D term is reported.
- `sent_command_*`, write-result and age fields describe serial `!G` writes.
  They do not prove that the MD applied the commands; compare `md_output_*`.
- Stop events record the reason mask, voltage, FF bits and timestamp before
  the driver sends `!EX`. Reasons include low voltage, MD fault, stale telemetry,
  startup stop, software request and rejected release. With `estop_latch_hardware=true`, a protection latch keeps
  its originating reason until an accepted manual release. The deployed default
  is false: physical/supply stops do not enable the software stop; supply
  recovery clears the hardware stop. Manually enabled software stops remain
  latched. Existing velocity publishers may resume motion after supply recovery. FF16 produced by our
  own `!EX` is not labelled as a new external MD fault. A physical switch that
  only cuts the supply still cannot be distinguished from other supply loss.

Diagnostics publication is configured for up to 50 Hz; use message timestamps
and measured rates when analyzing timing. Parameters are `publish_diagnostics`,
`diagnostics_topic` and `diagnostics_publish_hz`. All MD fields share one
periodic stream (CB/V/FF/A/BA/BS/T/FS/FM/P); standalone queries cancel this
controller's stream and must not be interleaved. The default 3 ms per slot
provides approximately 33 Hz per field. `diagnostics_poll_hz` is accepted only
for compatibility and has no effect. Streaming starts even with
`pub_odom_tf=false`; V/FF reception continues with `publish_status=false`.

The isolated PTY test at `test/test_protection_serial.py` exercises actual driver
parsing, unknown/stale MD fields, PI accounting, serial commands, stop reasons,
manual release and restart suppression. Source the workspace and run it with a
non-production `ROS_DOMAIN_ID` (for example `231`).
