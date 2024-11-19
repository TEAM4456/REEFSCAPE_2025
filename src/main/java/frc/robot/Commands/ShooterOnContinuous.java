
package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class ShooterOnContinuous extends Command{
    public final Shooter shooter;
    public ShooterOnContinuous(Shooter s){
        this.shooter = s;
    }
    public void execute() {
        shooter.shooterOn();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
      }
}
