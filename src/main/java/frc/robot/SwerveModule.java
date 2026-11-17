package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.Signal;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.telemetry.Telemetry;

public class SwerveModule {
  private final String name;
  private final double zeroOffsetRotations;

  private final SparkMax driveMotor;
  private final SparkMax turnMotor;
  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder turnEncoder;

  private final SimpleMotorFeedforward driveFeedforward =
      new SimpleMotorFeedforward(Constants.DRIVE_KS_VOLTS, Constants.DRIVE_KV_VOLTS_PER_MPS);
  private final PIDController drivePid = new PIDController(Constants.DRIVE_KP_VOLTS_PER_MPS, 0, 0);
  private final PIDController turnPid = new PIDController(Constants.TURN_KP_VOLTS_PER_RAD, 0, 0);

  public SwerveModule(Constants.ModuleConfig config) {
    name = config.name();
    zeroOffsetRotations = config.zeroOffsetRotations();

    driveMotor = new SparkMax(Constants.CAN_BUS, config.driveCanId(), MotorType.kBrushless);
    turnMotor = new SparkMax(Constants.CAN_BUS, config.turnCanId(), MotorType.kBrushless);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .inverted(config.driveInverted())
        .smartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    driveConfig.signals.primaryEncoderVelocityPeriodMs(20).primaryEncoderPositionPeriodMs(20);
    driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .idleMode(IdleMode.kBrake)
        .inverted(Constants.TURN_MOTOR_INVERTED)
        .smartCurrentLimit(Constants.TURN_CURRENT_LIMIT_AMPS)
        .voltageCompensation(12.0);
    turnConfig.absoluteEncoder.inverted(Constants.TURN_ENCODER_INVERTED);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(20)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20);
    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    driveEncoder = driveMotor.getEncoder();
    driveEncoder.setPosition(0.0);
    turnEncoder = turnMotor.getAbsoluteEncoder();

    turnPid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Rotation2d getAngle() {
    double rotations = turnEncoder.getPosition().get() - zeroOffsetRotations;
    return new Rotation2d(rotations * 2.0 * Math.PI);
  }

  public double getDriveVelocityMps() {
    double motorRpm = driveEncoder.getVelocity().get();
    return motorRpm / 60.0 / Constants.DRIVE_REDUCTION * 2.0 * Math.PI * Constants.WHEEL_RADIUS_METERS;
  }

  public double getDrivePositionMeters() {
    double motorRotations = driveEncoder.getPosition().get();
    return motorRotations / Constants.DRIVE_REDUCTION * 2.0 * Math.PI * Constants.WHEEL_RADIUS_METERS;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePositionMeters(), getAngle());
  }

  public SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(getDriveVelocityMps(), getAngle());
  }

  public void setDesiredVelocity(SwerveModuleVelocity desired) {
    Rotation2d current = getAngle();
    SwerveModuleVelocity target = desired.optimize(current).cosineScale(current);

    double driveVolts =
        driveFeedforward.calculate(target.velocity)
            + drivePid.calculate(getDriveVelocityMps(), target.velocity);
    double turnVolts = turnPid.calculate(current.getRadians(), target.angle.getRadians());

    driveMotor.setVoltage(clampVolts(driveVolts));
    turnMotor.setVoltage(clampVolts(turnVolts));
  }

  public void stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }

  public void logTelemetry() {
    String prefix = "Drive/" + name + "/";
    Signal<Double> rawAbsolute = turnEncoder.getPosition();
    Telemetry.log(prefix + "Raw Absolute (rot)", rawAbsolute.get());
    Telemetry.log(prefix + "Angle (deg)", getAngle().getDegrees());
    Telemetry.log(prefix + "Velocity (mps)", getDriveVelocityMps());
    Telemetry.log(prefix + "Turn Encoder OK", rawAbsolute.isValid());
    Telemetry.log(prefix + "Drive Encoder OK", driveEncoder.getVelocity().isValid());
  }

  private static double clampVolts(double volts) {
    return Math.max(-12.0, Math.min(12.0, volts));
  }
}
