
package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class FeedIn extends Command{
    public final Shooter shooter;
    public final Intake intake;
    public FeedIn(Shooter s, Intake i){
        this.shooter = s;
        this.intake = i;
    }
    public void execute() {
        shooter.feedBack();
        shooter.shooterIntake();
        
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        shooter.feedStop();
        shooter.shooterOff();
        
      }
}
