package frc.robot.Subsystems;

// Imports for the linear actuator

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


// Imports for the command system
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

// Imports for the constants
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {


  //Object declarations
  private SparkMax pivotElvMotor;
  private SparkMaxConfig pivotElvConfig;
  private RelativeEncoder pivotElvEncoder;
  private SparkClosedLoopController pivotElvPIDController;

  public ElevatorPivot() {
    //Setting the object's values
    pivotElvMotor = new SparkMax(15, MotorType.kBrushless);
    pivotElvEncoder = pivotElvMotor.getEncoder();

    //CONFIGURATIONS FOR Pivot Elevator MOTOR BELOW
    pivotElvConfig = new SparkMaxConfig();
    pivotElvConfig.idleMode(IdleMode.kBrake);
    pivotElvConfig.closedLoop.pidf(1,0,0,0);
    pivotElvConfig.openLoopRampRate(0.5);
    pivotElvConfig.smartCurrentLimit(40);
  }

  /*Manual Methods*/
  // Set's up the command for the elevator pivot movement
  public void elevatorPivotUp() {
    pivotElvMotor.set(-Constants.ElevatorPivotPositions.elevatorPivotSpeed);
  }
  public void elevatorPivotDown() {
    pivotElvMotor.set(Constants.ElevatorPivotPositions.elevatorPivotSpeed);
  }
  public void elevatorPivotStop() {
    pivotElvMotor.set(0);
  }

  /*Set Position Methods*/
  public void elevatorPivotCoralPickupPosition() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotCoralPickupPosition, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotClimbPosition() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotClimbPosition, SparkBase.ControlType.kPosition);
  }

  public void elevatorPivotScoreL1() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL1, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL2() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL2, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL3() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL3, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL4() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL4, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotAlgaeHigh() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotAlgaeHigh, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotAlgaeLow() {
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotAlgaeLow, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotDrivePosition(){
    pivotElvPIDController.setReference(Constants.ElevatorPivotPositions.elevatorPivotDrivePosition, SparkBase.ControlType.kPosition);
  }

  /*Create manually controlled commands here */
  // Run's the commands for elevator pivot movement
  public Command elevatorPivotUpCommand() {
    return run(() -> elevatorPivotUp());
  }
  public Command elevatorPivotDownCommand() {
    return run(() -> elevatorPivotDown());
  }
  public Command elevatorPivotStopCommand() {
    return run(() -> elevatorPivotStop());
  }

  /*Create set position commands here */


  public Command elevatorPivotCoralPickupPositionCommand() {
    return run(() -> elevatorPivotCoralPickupPosition()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotCoralPickupPosition) < 1));
  }
  public Command elevatorPivotClimbPositionCommand() {
    return run(() -> elevatorPivotClimbPosition()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotClimbPosition) < 1));
  }
  public Command elevatorPivotDrivePositionCommand() {
    return run(() -> elevatorPivotDrivePosition()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotDrivePosition) < 1));
  }

  public Command elevatorPivotScoreL1Command() {
    return run(() -> elevatorPivotScoreL1()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL1) < 1));
  }
  public Command elevatorPivotScoreL2Command() {
    return run(() -> elevatorPivotScoreL2()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL2) < 1));
  }
  public Command elevatorPivotScoreL3Command() {
    return run(() -> elevatorPivotScoreL3()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL3) < 1));
  }
  public Command elevatorPivotScoreL4Command() {
    return run(() -> elevatorPivotScoreL4()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL4) < 1));
  }
  public Command elevatorPivotAlgaeHighCommand() {
    return run(() -> elevatorPivotAlgaeHigh()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotAlgaeHigh) < 1));
  }
  public Command elevatorPivotAlgaeLowCommand() {
    return run(() -> elevatorPivotAlgaeLow()).until(() -> (Math.abs(pivotElvEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotAlgaeLow) < 1));
  }

  @Override
    public void periodic(){
      SmartDashboard.putNumber("elevatorPivotPosition",pivotElvEncoder.getPosition());
    }
}