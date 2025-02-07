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
  private RelativeEncoder pivotElvMotorEncoder;
  private final ClosedLoopConfig pivotElvPIDController;
  private SparkClosedLoopController pivotElvSet;

  public ElevatorPivot() {
    //Setting the object's values
    pivotElvMotor = new SparkMax(15, MotorType.kBrushless);
    pivotElvMotorEncoder = pivotElvMotor.getEncoder();
    pivotElvPIDController = new ClosedLoopConfig();
    pivotElvSet = pivotElvMotor.getClosedLoopController();

    //CONFIGURATIONS FOR Pivot Elevator MOTOR BELOW
    pivotElvConfig = new SparkMaxConfig();
    pivotElvConfig.idleMode(IdleMode.kBrake);
    pivotElvConfig.closedLoop.pid(1,0,0);
  }


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

  public void elevatorPivotCoralPickupPosition() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotIntakePosition, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotClimbPosition() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotClimbPosition, SparkBase.ControlType.kPosition);
  }

  public void elevatorPivotScoreL1() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL1, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL2() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL2, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL3() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL3, SparkBase.ControlType.kPosition);
  }
  public void elevatorPivotScoreL4() {
    pivotElvSet.setReference(Constants.ElevatorPivotPositions.elevatorPivotScoreL4, SparkBase.ControlType.kPosition);
  }


  // Run's the commands for elevator pivot movement
  public Command elevatorPivotUpCommand() {
    return run(() -> elevatorPivotUp()).withTimeout(0.1);
  }
  public Command elevatorPivotDownCommand() {
    return run(() -> elevatorPivotDown()).withTimeout(0.1);
  }
  public Command elevatorPivotStopCommand() {
    return run(() -> elevatorPivotStop());
  }

  public Command elevatorPivotCoralPickupPositionCommand() {
    return run(() -> elevatorPivotCoralPickupPosition());
  }
  public Command elevatorPivotClimbPositionCommand() {
    return run(() -> elevatorPivotClimbPosition());
  }

  public Command elevatorPivotScoreL1Command() {
    return run(() -> elevatorPivotScoreL1()).until(() -> (Math.abs(pivotElvMotorEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL1) < 1));
  }
  public Command elevatorPivotScoreL2Command() {
    return run(() -> elevatorPivotScoreL2()).until(() -> (Math.abs(pivotElvMotorEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL2) < 1));
  }
  public Command elevatorPivotScoreL3Command() {
    return run(() -> elevatorPivotScoreL3()).until(() -> (Math.abs(pivotElvMotorEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL3) < 1));
  }
  public Command elevatorPivotScoreL4Command() {
    return run(() -> elevatorPivotScoreL4()).until(() -> (Math.abs(pivotElvMotorEncoder.getPosition() - Constants.ElevatorPivotPositions.elevatorPivotScoreL4) < 1));
  }

  @Override
    public void periodic(){
      SmartDashboard.putNumber("elevatorPivotPosition",pivotElvMotorEncoder.getPosition());
    }
}