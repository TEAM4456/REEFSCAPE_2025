package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Elevator extends SubsystemBase {
    private SparkMax elevatorRight;
    private SparkMaxConfig elevatorRightEncoder;
    private SparkClosedLoopController elevatorRightPIDController;

    private SparkMax elevatorLeft;
    private SparkMaxConfig elevatorLeftEncoder;
    private SparkClosedLoopController elevatorLeftPIDController;

  public Elevator() {
    elevatorRight = new SparkMax(15, MotorType.kBrushless);
    elevatorLeft = new SparkMax(14, MotorType.kBrushless);
  }
//     elevatorRight = new SparkMax(15,MotorType.kBrushless);
//     elevatorRight.setOpenLoopRampRate(.5);
//     elevatorRightEncoder = elevatorRight.getEncoder();
//     elevatorRightPIDController = elevatorRight.getSparkClosedLoopController();

//     elevatorLeft = new SparkMax(14,MotorType.kBrushless);
//     elevatorLeft.setOpenLoopRampRate(.5);
//     elevatorLeftEncoder = elevatorLeft.getEncoder();
//     elevatorLeftPIDController = elevatorLeft.getPIDController();

//     elevatorRightPIDController.pid(1, 0, 0);
//     elevatorRightPIDController.setFF(0);

//     elevatorLeftPIDController.pid(1, 0, 0);
//     elevatorLeftPIDController.setFF(0);

//   }
    
//   public void elevatorUp(){
//     elevatorRight.set(-Constants.ElevatorPositions.elevatorSpeed);
//     elevatorLeft.set(Constants.ElevatorPositions.elevatorSpeed);
//   }
//   public void elevatorDown(){
//     elevatorRight.set(Constants.ElevatorPositions.elevatorSpeed);
//     elevatorLeft.set(-Constants.ElevatorPositions.elevatorSpeed);
//   }
//   public void elevatorStop(){
//     elevatorRight.set(0);
//     elevatorLeft.set(0);
//     elevatorLeftPIDController.setReference(elevatorLeftEncoder.getPosition(), SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(elevatorRightEncoder.getPosition(), SparkMax.ControlType.kPosition);
//   }





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
