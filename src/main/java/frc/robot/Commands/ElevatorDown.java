package frc.robot.Commands;



import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Intake;

public class ElevatorDown extends Command{
    public final Elevator elevator;
    public ElevatorDown(Elevator e){
        this.elevator = e;
    }
    public void execute() {
        elevator.elevatorDown();
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        elevator.elevatorStop();
      }
}