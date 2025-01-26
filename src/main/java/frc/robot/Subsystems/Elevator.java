package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//test

public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder; //maybe not needed with 2025 migration
    private SparkClosedLoopController elevatorPIDController;
    private SparkMaxConfig elevatorConfig;

  public Elevator() {
    SparkMax elevatorMotor = new SparkMax(18, MotorType.kBrushless);
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    elevatorConfig
      .idleMode(IdleMode.kBrake);
    elevatorConfig.closedLoop
      .pid(1,0,0);

   /* look into getting rid of this? Not sure if needed? Sat 1/25
   
   elevatorMotor.setOpenLoopRampRate(.5);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorPIDController = elevatorMotor.getClosedLoopController();
    

     elevatorPIDController.pid(1, 0, 0);
     elevatorPIDController.setFF(0); */

  

   }
  
  //When called, this moves the moter at negative elevatorSpeed to go up
  public void elevatorUp(){
    elevatorMotor.set(-Constants.ElevatorPositions.elevatorSpeed);
  }
  //When called, this moves the moter at positive elevatorSpeed to go down
  public void elevatorDown(){
    elevatorMotor.set(Constants.ElevatorPositions.elevatorSpeed);
  }
   public void elevatorStop(){
     elevatorMotor.set(0);

//     elevatorLeftPIDController.setReference(elevatorLeftEncoder.getPosition(), SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(elevatorRightEncoder.getPosition(), SparkMax.ControlType.kPosition);
/  }





//   public void setElevatorPositionUp(){
//     elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorUp, SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorUp, SparkMax.ControlType.kPosition);
//   }
//   public void setElevatorPositionDown(){
//     elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorDown, SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorDown, SparkMax.ControlType.kPosition);
//   }
//   public Command setElevatorPositionUpCommand(){
//     return run(() -> setElevatorPositionUp()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorUp) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorUp) < 1));
//   }
//   public Command setElevatorPositionDownCommand(){
//     return run(() -> setElevatorPositionDown()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorDown) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorDown) < 1));
//   }
//   @Override
//   public void periodic() {
//     SmartDashboard.putNumber("elevatorPositionRight",elevatorRightEncoder.getPosition());
//     SmartDashboard.putNumber("elevatorPositionRight", elevatorLeftEncoder.getPosition());
}
*/