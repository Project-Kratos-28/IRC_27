# IRC_27 — Kratos rover motherboard firmware

Firmware for the rover motherboard (STM32H753ZI, FreeRTOS + micro-ROS). It runs:

- **Drive**: 4-wheel independent steer, with 4 drive motors and 4 steering motors.
- **Arm**: base, shoulder, elbow, a 2-motor bevel wrist (pitch/roll mixing), linear actuator and gripper.

The board is a micro-ROS client. It connects over serial to a micro-ROS agent
on the rover computer. The commands come from:

- the drive high level ([electronics_drive_high_level](https://github.com/Project-Kratos-28/electronics_drive_high_level), branch `dev`)
- the arm/payload high level

The steering PID and the tuning tool come from
[electronics_pid_class](https://github.com/Project-Kratos-28/electronics_pid_class) (branch `steer-tuner`).

```
joystick / Nav2 ──► /cmd_vel ──► drive_controller.py ──► /drive_controller/rover ──► micro-ROS agent ══serial══► STM32
                                (high level, ROS 2)       Float64MultiArray[8]                                   drive.cpp
```

---

## Repository layout

| Path | What it is |
|---|---|
| `motherboard_kratos/motherboard_kratos.ioc` | CubeMX config (pins, timers, FreeRTOS tasks). The source of truth for hardware. |
| `motherboard_kratos/Core/Src/drive.cpp`, `Inc/drive.h` | 4-wheel independent-steer drive: config table, control loop and safety. |
| `Core/Src/motor_control.cpp` | `motor_control` task entry. Runs the drive and the arm motors. |
| `Core/Src/microros_main.c` | micro-ROS: agent connection state machine, subscribers, arm mixing. |
| `Core/Src/SpeedMotor.cpp` | Closed-loop speed control (drive wheels): PWM plus a single DIR pin. |
| `Core/Src/SteerMotor.cpp`, `PositionMotor.cpp` | Closed-loop steering angle control on an AS5600 absolute encoder. |
| `Core/Src/PID.cpp` | Generic PID: anti-windup, deadband with hysteresis, filtered D. |
| `Core/Src/freertos.c` | CubeMX-generated tasks. Only the `USER CODE` blocks are edited. |
| `micro_ros_stm32cubemx_utils/` | Prebuilt micro-ROS static library and transport (includes the custom messages). |

---

## ROS interface

Node name: `rover_node`.

| Topic | Type | Direction | Use |
|---|---|---|---|
| `/drive_controller/rover` | `std_msgs/Float64MultiArray` | sub | Drive command (see below) |
| `/payload_cmd` | `kratos_teerth_rohan_pandya_msgs/ArmCmd` | sub | Arm command (`use_arm`, `arm_cmd[7]`) |
| `/misc_topic` | `std_msgs/Bool` | sub | Reserved |

**Drive command**: 8 values, wheel order **FL, FR, RL, RR**:

| Index | Meaning | Units |
|---|---|---|
| `0..3` | Steering angle | deg, `0..180`, **90 = straight ahead**, `>90` = wheel turned right |
| `4..7` | Wheel speed | rpm, `+` = rover moves forward |

Leave the array `layout` empty, as the high level does. The firmware sets aside no
storage for layout dimensions.

---

## Drive hardware map

Matches the fabricated motherboard: [Electronics_Motherboard](https://github.com/Project-Kratos-28/Electronics_Motherboard),
branch `NEW_MOTHERBOARD_FILES_dev`, file `motherboard - kratos/motherboard - kratos.kicad_pcb`.
Each wheel uses one 5-pin `drive_conn_set` terminal: pins 1/2 are drive PWM/DIR,
pins 3/4 are steer PWM/DIR, and pin 5 is GND.

| Wheel | Terminal | Drive PWM | Drive DIR | Drive encoder | Steer PWM | Steer DIR | AS5600 |
|---|---|---|---|---|---|---|---|
| FL | J38 `set_1` | TIM5_CH1 `PA0` | `PG2` | J25 TIM1 `PE9`/`PE11` | TIM5_CH2 `PA1` | `PG3` | J17, mux ch0 |
| FR | J39 `set_2` | TIM5_CH3 `PA2` | `PG4` | J26 TIM2 `PA5`/`PB3` | TIM15_CH2 `PE6` | `PG5` | J16, mux ch1 |
| RL | J40 `set_3` | TIM16_CH1 `PF6` | `PG6` | J27 TIM3 `PA6`/`PA7` | TIM17_CH1 `PF7` | `PG7` | J15, mux ch2 |
| RR | J41 `set_4` | TIM13_CH1 `PF8` | `PG8` | J28 TIM4 `PD12`/`PD13` | TIM14_CH1 `PF9` | `PG9` | J14, mux ch3 |

- **Assumed, check on the rover:** `set_N` is wheel N in FL, FR, RL, RR order; pins 1/2
  are drive and 3/4 are steer; `AB_enc_conn_N` and `abs_enc_conn_N` belong to wheel N.
  If a wheel is wired differently, change only its row in `WHEEL_CFG`.
- **Drive encoders**: quadrature, 13 PPR motor encoder with a 181:1 gearbox. The x2/x4
  edge multiplier is read from each timer's encoder mode at startup. TIM2 is x4 and
  TIM1/3/4 are x2.
- **Steering encoders**: AS5600 (I2C address `0x36`) behind a TCA9548A mux (`0x70`) on
  **I2C1** (`PB6` SCL / `PB7` SDA), about 100 kHz. Channels 4 and 5 (J30/J31) are spare,
  and channels 6 and 7 are the IMU connectors.
- **PWM**: every timer runs at ARR 65535, which gives about 977 Hz.

### Known board defects (fabricated rev 1)

- **The Nucleo GND pads are not connected to board GND** (no copper). Add a wire from a
  Nucleo GND pin to board GND, otherwise PWM/DIR/I2C have no common reference.
- **The TCA9548A `Vin` pad is unconnected.** Wire it to 3V3, otherwise no steering
  encoder can be read and all steering (and therefore drive) stays disabled.
- The arm DIR lines on J6 pin 4 / J7 pin 2 are `PD8`/`PD9`, the same pins as USART3
  (micro-ROS serial). The firmware moved those DIR signals to `PB8`/`PB9`, which the
  board does not route.

### Arm hardware map (firmware, for reference)

| Joint | PWM | DIR |
|---|---|---|
| Bevel 1 | TIM8_CH1 `PC6` | `PD7` |
| Bevel 2 | TIM8_CH2 `PC7` | `PB8` |
| Shoulder | TIM8_CH3 `PC8` | `PB9` |
| Elbow | TIM8_CH4 `PC9` | `PD10` |
| Base | TIM15_CH1 `PE5` | `PD11` |

Arm polarity and maximum speed macros are at the top of `microros_main.c`.

---

## How the drive works

`motor_control` task (priority High7, 100 Hz, measures its own `dt`):

1. Takes a copy of the latest drive command, which the micro-ROS callback stores via `Drive_SetCommand()`.
2. Reads all 4 AS5600 encoders. This takes about 2.4 ms.
3. Reads the drive encoders. Speed is averaged over the last 5 cycles (50 ms).
4. **Steering**: each wheel's PID runs in degrees. Each steering target can change by
   at most 90 °/s, so the 4 wheels stay in step during a swing.
5. **Steering gate**: drive speed is scaled down while any wheel is far from its target
   angle. It is full speed below 10° of error and zero above 30°.
6. **Drive**: each wheel's PID runs in rpm. A per-wheel direction sign accounts for
   mirrored motors.

### Safety behaviour

| Condition | Response |
|---|---|
| No drive command for **500 ms**, or agent disconnect | Drive PWM off. Steering holds its last target. |
| Before the first command after boot | Steering holds the position read at boot (no jump). Drive off. |
| AS5600 fails 10 reads in a row | That steering motor turns off and drive stops. Clears itself once readings come back. |
| Measured steering angle more than 15° outside `0..180` | Steering motor turns off, latched until reset. This catches a reversed motor or encoder direction. |
| NaN/inf in a command | Whole message ignored |

### Voltage clamp

The battery (22 V) is above both motors' rated voltage. Duty is capped at
`V_rated / V_supply`:

| Motor | Rated | Max duty |
|---|---|---|
| Drive | 18 V | 81.8 % |
| Steer | 12 V | 54.5 % |

The cap is applied to the PWM output **and** used as the PID output limit, so the
integral term cannot wind up against it. Change `SUPPLY_VOLTAGE`, `DRIVE_MOTOR_RATED_V`
and `STEER_MOTOR_RATED_V` in `drive.cpp`.

> A 6S LiPo is 22.2 V nominal but **25.2 V fully charged**. Set `SUPPLY_VOLTAGE` to
> 25.2 if the motors must never see more than their rating on average.

---

## Calibration (required before driving)

All of these are in the `WHEEL_CFG` table in `Core/Src/drive.cpp`.

| Field | How to find it |
|---|---|
| `driveMotorDir` | `+1` if DIR **high** drives the rover **forward**. The left and right sides are usually opposite. |
| `driveEncDir` | `+1` if the encoder count goes **up** when the rover drives forward. |
| `steerHomeRaw` | Point the wheel straight and read `g_steer_raw[i]`. |
| `steerEncDir` | `+1` if raw counts go **up** as the wheel turns **right** (angle increases). |
| `steerMotorDir` | `+1` if DIR high turns the wheel right. Use the tuner's *dircheck*. |
| `steerMuxCh` | The PCA9548A channel this wheel's AS5600 is on. |

> ⚠️ If `driveMotorDir` and `driveEncDir` disagree for a wheel, its speed loop pushes the
> wrong way and the wheel **runs away at the duty cap**. Do the first runs with the
> wheels off the ground.

Gains (defaults in `drive.cpp`; can be changed live over SWD):

| | kp | ki | kd | Notes |
|---|---|---|---|---|
| Drive (per rpm) | 10 | 8 | 0.1 | From the previous 6-wheel firmware |
| Steer (per degree) | 2 | 0 | 0.05 | **Placeholder**: tune with `pid_tuner_position.py` |

---

## Build, flash, run

**Toolchain**: STM32CubeIDE (2.2.0 tested). Import `motherboard_kratos/` as an existing
project and build the `Debug` configuration. Headless:

```bash
/path/to/stm32cubeide/headless-build.sh -data /tmp/cubews \
    -import /path/to/IRC_27/motherboard_kratos -build motherboard_kratos/Debug
```

**Flash** (ST-Link, OpenOCD):

```bash
cd motherboard_kratos
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program Debug/motherboard_kratos.elf verify reset exit"
```

**micro-ROS agent**: USART3 (`PD8`/`PD9`), 115200 baud. On a Nucleo this is the
ST-Link virtual COM port, usually `/dev/ttyACM0`:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
ros2 node list          # expect /rover_node
```

**Bench test without the high level** (wheels off the ground):

```bash
# all wheels straight, 10 rpm
ros2 topic pub -r 20 /drive_controller/rover std_msgs/msg/Float64MultiArray \
    "{data: [90,90,90,90, 10,10,10,10]}"
# stop the publisher -> drive must stop within 0.5 s
```

**Full stack**: in `electronics_drive_high_level` (branch `dev`):

```bash
ros2 launch drive_controls rover.launch.py
```

Lower `max_wheel_rpm` in `config/rover_params.yaml` for the first runs.

### Debugging

Add these to STM32CubeIDE *Live Expressions* (or read them over SWD):

| Variable | Meaning |
|---|---|
| `g_steer_raw[4]` / `g_steer_deg[4]` / `g_steer_target_deg[4]` | AS5600 raw count / measured angle / target angle |
| `g_drive_rpm[4]` / `g_drive_target_rpm[4]` | Measured and commanded wheel speed (rover frame) |
| `g_drive_duty[4]` / `g_steer_duty[4]` | PID output, % |
| `g_steer_enc_ok`, `g_steer_enc_err[4]` | Encoder read status / error counts |
| `g_steer_fault` | Bit n = wheel n steering disabled |
| `g_drive_cmd_age_ms` | Time since the last drive command (`0xFFFFFFFF` = none yet) |
| `g_drive_speed_scale` | 0..1 drive scale from the steering gate |
| `g_drive_kp/ki/kd`, `g_steer_kp/ki/kd` | Gains. Writable live. |

---

## Notes / constraints

- **Do not change the `.ioc` casually.** CubeMX regeneration overwrites everything
  outside the `USER CODE` blocks and can wipe `Drivers/` (restore it with
  `git checkout -- motherboard_kratos/Drivers`).
- The `motor_control` task stack is 128 words (set in the `.ioc`). The worst-case drive
  path uses about 455 B at `-O0`, including the FPU exception frame. Raise it to 256 words
  next time the `.ioc` is edited.
- `rover_app.c` at the project root is an old skeleton and is **not built**.
