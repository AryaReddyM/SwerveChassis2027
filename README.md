# SwerveChassis2027

A minimal swerve chassis for the 2027 Systemcore emulator (Raspberry Pi 5). It uses the **practice robot** hardware from `Rebuilt2026`, branch `lockedShooter`, file `robots/practice/PracticeDrive.java`.

| | Practice robot (copied here) |
|---|---|
| Drive | SPARK MAX + NEO, CAN IDs FL 4, FR 5, BL 6, BR 8 (FL and BL inverted) |
| Turn | SPARK MAX + NEO, CAN IDs FL 3, FR 7, BL 2, BR 41 |
| Steering sensor | Through Bore absolute encoder on each turn SPARK MAX's data port |
| Gearing | Drive 7.31:1, turn 12.8:1, 2" wheel radius, 0.7996 m square wheelbase |
| Gyro | navX on USB in 2026. **Not used here:** there is no navX library for 2027 yet, and the Pi has no IMU, so the robot drives robot-relative. |

Versions: WPILib **2027.0.0-alpha-8**, GradleRIO `org.wpilib.GradleRIO` 2027.0.0-alpha-8, Java 25, REVLib **2027.0.0-alpha-9**.

> **Status:** these files have not been compiled yet. This PC only has WPILib 2026 (JDK 17), and 2027 needs JDK 25. Every API call was checked against the Alpha 8 / REVLib alpha-9 sources, but run `gradlew build` first.

## Setup (once)

1. Install the **WPILib 2027 Alpha 8** installer: <https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-8>. It installs alongside 2026 and includes JDK 25 and the 2027 VS Code.
2. Open this folder in the **2027** WPILib VS Code.
3. Run **WPILib: Build Robot Code**, or `./gradlew build`.

## Hardware on the Pi

- **USB-CAN adapter** (candleLight / gs_usb). Plug it in and it becomes `can_s0`, which is `Constants.CAN_BUS = CANPort.CAN_S0`.
- **Pico 2 (RP2350)**: not needed for this chassis. Everything here is on CAN. The Pico is for PWM/DIO/analog.
- On first power-up, check that the SPARK MAXes are on 2026 firmware (MAX v26.1.5). REV says 2027 uses the 2026 firmware.

## Deploy

Connect to the Pi's Wi-Fi (`SYSTEMCORE-…`) or the same network, then run **WPILib: Deploy Robot Code** (`./gradlew deploy`).

GradleRIO 2027 does the following:
- Logs in as `systemcore` and tries `robot.local`, then `172.30.0.1`.
- Copies the jar and natives.
- Writes `/home/systemcore/robotCommand`.
- Runs `sudo systemctl start robot`.

To watch the robot's output: `ssh systemcore "journalctl -u robot -f"`

Enabling the robot needs a Driver Station: the **2027 FIRST Driver Station alpha (≥ alpha 7)** or the NI DS.

## Bring-up order (wheels off the ground!)

1. **Calibrate zero offsets.** Every offset is `0.0`; the practice robot was never calibrated in Rebuilt2026 either.
   1. Point all wheels straight forward.
   2. Read `Drive/<module>/Raw Absolute (rot)` in Elastic (`http://robot.local:5803`) or AdvantageScope Lite.
   3. Paste the values into `Constants.java`.
2. Enable teleop. With `TELEOP_SPEED_MPS` at 1.5 m/s, check each of these:
   - Pushing the left stick forward moves the robot forward. If not, flip `driveInverted` per module.
   - The wheels steer to the stick direction without oscillating. If they run away, flip `TURN_ENCODER_INVERTED` or `TURN_MOTOR_INVERTED`.
   - Right stick right turns the robot clockwise.
3. Tune `TURN_KP_VOLTS_PER_RAD`, then `DRIVE_KV_VOLTS_PER_MPS` and `DRIVE_KP_VOLTS_PER_MPS`.

## 2026 → 2027 changes you'll see in this code

| 2026 | 2027 |
|---|---|
| `edu.wpi.first.*` packages | `org.wpilib.*` (for example `org.wpilib.framework.TimedRobot`, `org.wpilib.math.*`) |
| `RobotBase.startRobot(Robot.class)` | `RobotBase.startRobot(Robot::new)` |
| `XboxController`, A/B/X/Y buttons | `Gamepad`, FaceDown/FaceRight/FaceLeft/FaceUp buttons |
| `ChassisSpeeds`, `SwerveModuleState` | `ChassisVelocities`, `SwerveModuleVelocity` |
| `toSwerveModuleStates`, `desaturateWheelSpeeds` | `toWheelVelocities`, `desaturateWheelVelocities` |
| `Rotation2d.kZero` | `Rotation2d.ZERO` |
| `SmartDashboard.putNumber` | `Telemetry.log` |
| Test mode | Utility mode |
| Gyro from a vendor (navX/Pigeon) | `OnboardIMU`, built into Systemcore |
| `new SparkMax(id, type)` | `new SparkMax(CANPort.CAN_S0, id, type)` |
| SPARK `positionConversionFactor(...)` | Removed: encoders report rotations/RPM; convert in code |
| `encoder.getPosition()` returns `double` | Returns `Signal<Double>`: use `.get()` and `.isValid()` |
| `SparkBase.ControlType` | `SparkLowLevel.ControlType` |
