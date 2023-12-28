package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  // private Joystick controller;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private boolean fieldOriented = false;
  private boolean onePress = false;

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    driver = new XboxController(Constants.JoystickConstants.kDriverControllerPort);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    if (driver.getRightBumper()) {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY() * 0.5, Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX() * 0.5, Constants.Swerve.axisDeadBand));
      rotationVal =
          rotationLimiter.calculate(
              MathUtil.applyDeadband(driver.getRightX() * 0.25, Constants.Swerve.axisDeadBand));
    } else {
      translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftY(), Constants.Swerve.axisDeadBand));
      strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(driver.getLeftX(), Constants.Swerve.axisDeadBand));        
      rotationVal =
          rotationLimiter.calculate(
              MathUtil.applyDeadband(driver.getRightX() * 0.5, Constants.Swerve.axisDeadBand));
    }
    
    if (driver.getLeftBumperPressed() && onePress==false) {
      fieldOriented = !fieldOriented;
      onePress = true;
    }else if(driver.getLeftBumperReleased()) {
      onePress = false;
    }
    if (driver.getBackButton()) s_Swerve.zeroGyro(); 
    // if (driver.getRawButton(Constants.JoystickConstants.btn_minus)) s_Swerve.resetOdometry(new Pose2d());
    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity, fieldOriented,
        true);

    SmartDashboard.putBoolean("isOriented", fieldOriented);
  }
}
