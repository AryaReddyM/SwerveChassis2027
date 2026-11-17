package frc.robot;

import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.telemetry.Telemetry;

public class Drivetrain {
  private final SwerveModule frontLeft = new SwerveModule(Constants.FRONT_LEFT);
  private final SwerveModule frontRight = new SwerveModule(Constants.FRONT_RIGHT);
  private final SwerveModule backLeft = new SwerveModule(Constants.BACK_LEFT);
  private final SwerveModule backRight = new SwerveModule(Constants.BACK_RIGHT);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(Constants.HALF_TRACK_METERS, Constants.HALF_TRACK_METERS),
          new Translation2d(Constants.HALF_TRACK_METERS, -Constants.HALF_TRACK_METERS),
          new Translation2d(-Constants.HALF_TRACK_METERS, Constants.HALF_TRACK_METERS),
          new Translation2d(-Constants.HALF_TRACK_METERS, -Constants.HALF_TRACK_METERS));

  private final OnboardIMU imu =
      Constants.USE_ONBOARD_IMU ? new OnboardIMU(OnboardIMU.MountOrientation.FLAT) : null;

  public Drivetrain() {
    if (imu != null) {
      imu.resetYaw();
    }
  }

  public boolean hasGyro() {
    return imu != null;
  }

  public Rotation2d getHeading() {
    return imu != null ? imu.getRotation2d() : Rotation2d.ZERO;
  }

  public void resetHeading() {
    if (imu != null) {
      imu.resetYaw();
    }
  }

  public void drive(
      double xMps, double yMps, double omegaRadPerSec, boolean fieldRelative, double periodSeconds) {
    var chassis = new ChassisVelocities(xMps, yMps, omegaRadPerSec);
    if (fieldRelative && imu != null) {
      chassis = chassis.toRobotRelative(getHeading());
    }
    chassis = chassis.discretize(periodSeconds);

    SwerveModuleVelocity[] targets =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toWheelVelocities(chassis), Constants.MAX_SPEED_MPS);

    frontLeft.setDesiredVelocity(targets[0]);
    frontRight.setDesiredVelocity(targets[1]);
    backLeft.setDesiredVelocity(targets[2]);
    backRight.setDesiredVelocity(targets[3]);
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  public void logTelemetry() {
    frontLeft.logTelemetry();
    frontRight.logTelemetry();
    backLeft.logTelemetry();
    backRight.logTelemetry();
    Telemetry.log("Drive/Has Gyro", hasGyro());
    Telemetry.log("Drive/Heading (deg)", getHeading().getDegrees());
  }
}
