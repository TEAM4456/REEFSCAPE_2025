package frc.robot.Commands;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;


public class ElevatorUp extends Command{
    public final Elevator elevator;
    public ElevatorUp(Elevator e){
        this.elevator = e;
    }
    public void execute() {
        elevator.elevatorUp();
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
        elevator.elevatorStop();
      }
}