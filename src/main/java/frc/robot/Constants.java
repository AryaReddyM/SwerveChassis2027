package frc.robot;

import org.wpilib.hardware.bus.CANPort;

public final class Constants {
  private Constants() {}

  public static final CANPort CAN_BUS = CANPort.CAN_S0;

  public record ModuleConfig(
      String name, int driveCanId, int turnCanId, double zeroOffsetRotations, boolean driveInverted) {}

  public static final ModuleConfig FRONT_LEFT = new ModuleConfig("Front Left", 4, 3, 0.0, true);
  public static final ModuleConfig FRONT_RIGHT = new ModuleConfig("Front Right", 5, 7, 0.0, false);
  public static final ModuleConfig BACK_LEFT = new ModuleConfig("Back Left", 6, 2, 0.0, true);
  public static final ModuleConfig BACK_RIGHT = new ModuleConfig("Back Right", 8, 41, 0.0, false);

  public static final double HALF_TRACK_METERS = 0.3998;
  public static final double WHEEL_RADIUS_METERS = 0.0508;

  public static final double DRIVE_REDUCTION = 7.31;
  public static final int DRIVE_CURRENT_LIMIT_AMPS = 35;
  public static final double NEO_FREE_SPEED_RPM = 5676.0;

  public static final double DRIVE_FREE_SPEED_MPS =
      NEO_FREE_SPEED_RPM / 60.0 / DRIVE_REDUCTION * 2.0 * Math.PI * WHEEL_RADIUS_METERS;

  public static final double MAX_SPEED_MPS = 3.5;
  public static final double TELEOP_SPEED_MPS = 1.5;
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI;

  public static final double DRIVE_KS_VOLTS = 0.0;
  public static final double DRIVE_KV_VOLTS_PER_MPS = 12.0 / DRIVE_FREE_SPEED_MPS;
  public static final double DRIVE_KP_VOLTS_PER_MPS = 0.0;

  public static final int TURN_CURRENT_LIMIT_AMPS = 10;
  public static final boolean TURN_MOTOR_INVERTED = true;
  public static final boolean TURN_ENCODER_INVERTED = false;
  public static final double TURN_KP_VOLTS_PER_RAD = 0.005 * Math.toDegrees(1.0) * 12.0;

  public static final boolean USE_ONBOARD_IMU = false;

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final double JOYSTICK_DEADBAND = 0.08;
}
