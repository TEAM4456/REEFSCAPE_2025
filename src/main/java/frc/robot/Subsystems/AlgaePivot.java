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

public class AlgaePivot extends SubsystemBase {


  //variable declarations
  private SparkMax pivotAlgMotor;
  private SparkMaxConfig pivotAlgConfig;
  private RelativeEncoder pivotAlgEncoder;
  private SparkClosedLoopController pivotAlgPIDController;

  public AlgaePivot() {
    //Setting the motor's values
    pivotAlgMotor = new SparkMax(23, MotorType.kBrushless);
    pivotAlgEncoder = pivotAlgMotor.getEncoder();

    //CONFIGURATIONS FOR Pivot Algae MOTOR BELOW
    pivotAlgConfig = new SparkMaxConfig();
    pivotAlgConfig.idleMode(IdleMode.kBrake);
    pivotAlgConfig.closedLoop.pidf(1,0,0,0);
    pivotAlgConfig.openLoopRampRate(0.5);
    pivotAlgConfig.smartCurrentLimit(40);
  }

  /*Manual Methods*/
  // Set's up the command for the algae pivot movement
  public void algaePivotUp() {
    pivotAlgMotor.set(-Constants.AlgaePivotPositions.algaePivotSpeed);
  }
  public void algaePivotDown() {
    pivotAlgMotor.set(Constants.AlgaePivotPositions.algaePivotSpeed);
  }
  public void algaePivotStop() {
    pivotAlgMotor.set(0);
  }

  /*Set Position Methods*/ 
  public void setAlgaePivotUp() {
    pivotAlgPIDController.setReference(Constants.AlgaePivotPositions.algaePivotUp, SparkBase.ControlType.kPosition);
  }
  public void setAlgaePivotDown() {
    pivotAlgPIDController.setReference(Constants.AlgaePivotPositions.algaePivotDown, SparkBase.ControlType.kPosition);
    }
  

  /*Create manually controlled commands here */
  // Run's the commands for algae pivot movement
  public Command algaePivotUpCommand() {
    return run(() -> algaePivotUp());
  }
  public Command algaePivotDownCommand() {
    return run(() -> algaePivotDown());
  }
  public Command algaePivotStopCommand() {
    return run(() -> algaePivotStop());
  }

  /*Create set position commands here */

  public Command setAlgaePivotUpCommand() {
    return run(() -> setAlgaePivotUp()).until(() -> (Math.abs(pivotAlgEncoder.getPosition() - Constants.AlgaePivotPositions.algaePivotUp) < 1));
  }
  public Command setAlgaePivotDownCommand() {
    return run(() -> setAlgaePivotDown()).until(() -> (Math.abs(pivotAlgEncoder.getPosition() - Constants.AlgaePivotPositions.algaePivotDown) < 1));
  }

  @Override
    public void periodic(){
      SmartDashboard.putNumber("algaePivotPosition",pivotAlgEncoder.getPosition());
    }
}