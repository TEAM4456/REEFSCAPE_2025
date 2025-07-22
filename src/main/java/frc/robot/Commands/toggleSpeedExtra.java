package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve;
//import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class toggleSpeedExtra extends Command {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
//  private BooleanSupplier robotCentricSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(6.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(6.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(60);

  public toggleSpeedExtra(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup
      /*BooleanSupplier robotCentricSup*/) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    //this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal/10, strafeVal/10).times(Constants.Swerve.maxSpeed),
        (rotationVal/10) * Constants.Swerve.maxAngularVelocity,
        //!robotCentricSup.getAsBoolean(),
        true);
  }
  public void end(boolean interupted){};
}