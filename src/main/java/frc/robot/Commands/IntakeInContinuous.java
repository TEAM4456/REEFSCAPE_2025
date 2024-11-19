
package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class IntakeInContinuous extends Command{
    public final Intake intake;
    public IntakeInContinuous(Intake i){
        this.intake = i;
    }
    public void execute() {
        intake.speedForward();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
      }
}