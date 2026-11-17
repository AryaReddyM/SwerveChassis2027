package frc.robot;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.framework.TimedRobot;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathUtil;

public class Robot extends TimedRobot {
  private final Gamepad driver = new Gamepad(Constants.DRIVER_CONTROLLER_PORT);
  private final Drivetrain drivetrain = new Drivetrain();

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotPeriodic() {
    drivetrain.logTelemetry();
  }

  @Override
  public void teleopPeriodic() {
    if (driver.getFaceDownButtonPressed()) {
      drivetrain.resetHeading();
    }

    double x =
        -xLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY(), Constants.JOYSTICK_DEADBAND))
            * Constants.TELEOP_SPEED_MPS;
    double y =
        -yLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX(), Constants.JOYSTICK_DEADBAND))
            * Constants.TELEOP_SPEED_MPS;
    double rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(driver.getRightX(), Constants.JOYSTICK_DEADBAND))
            * Constants.MAX_ANGULAR_SPEED_RAD_PER_SEC;

    drivetrain.drive(x, y, rot, drivetrain.hasGyro(), getPeriod());
  }

  @Override
  public void autonomousInit() {
    drivetrain.stop();
  }

  @Override
  public void disabledInit() {
    drivetrain.stop();
  }
}
