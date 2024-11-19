
package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePulley;

public class MoveIntakeIn extends Command{
    public final IntakePulley intakepulley;
    public MoveIntakeIn(IntakePulley i){
        this.intakepulley = i;
    }
    public void execute() {
        intakepulley.moveIntakeIn();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        intakepulley.moveIntakeStop();
      }
}