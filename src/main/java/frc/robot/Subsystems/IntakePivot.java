/*
Below is copied exactly from elevator.java
*/
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
import frc.robot.Constants.ElevatorPositions;

public class IntakePivot extends SubsystemBase {
    private SparkMax intakePivotMotor;
    private RelativeEncoder intakePivotEncoder; 
    private SparkClosedLoopController intakePivotPIDController;
    private SparkMaxConfig intakePivotConfig;

    public IntakePivot() {
        intakePivotMotor = new SparkMax(19, MotorType.kBrushless);
        intakePivotEncoder = intakePivotMotor.getEncoder();
        intakePivotConfig = new SparkMaxConfig();
        intakePivotConfig.idleMode(IdleMode.kBrake);
        intakePivotConfig.closedLoop.pid(1,0,0);
}

public class intakePivot extends SubsystemBase {
    private SparkMax intakePivotMotor;
    private RelativeEncoder intakePivotEncoder; 
    private SparkClosedLoopController intakePivotPIDController;
    private SparkMaxConfig intakePivotConfig;

    /*  // look into getting rid of this? Not sure if needed? Sat 1/25
   //elevatorMotor.setOpenLoopRampRate(.5);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorPIDController = elevatorMotor.getClosedLoopController();

     //elevatorPIDController.pid(1, 0, 0);
     elevatorPIDController.setFF(0); */
   }
  
  /* Create your elevator Methods (? correct terminology) here */

  //When called, this moves the motor at negative elevatorSpeed to go up
  public void intakePivotUp(){
    intakePivotMotor.set(-Constants.ElevatorPositions.elevatorSpeed);
  }
  //When called, this moves the moter at positive elevatorSpeed to go down
  public void intakePivotDown(){
    intakePivotMotor.set(Constants.ElevatorPositions.elevatorSpeed);
  }
   public void intakePivotStop(){
    intakePivotMotor.set(0);
}

//format: subsystemScoreL1
  public void intakePivotScoreL1()
  {
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScorel1, SparkMax.ControlType.kPosition);
  }
  public void intakePivotScoreL2()
  {
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScorel2, SparkMax.ControlType.kPosition);
  }
  public void intakePivotScoreL3()
  {
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScorel3, SparkMax.ControlType.kPosition);
  }
  public void intakePivotScoreL4()
  {
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScorel4, SparkMax.ControlType.kPosition);
  }
  public void intakePivotCoralPickup()
  {
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotCoralPickup, SparkMax.ControlType.kPosition);
  }
  


//     elevatorLeftPIDController.setReference(elevatorLeftEncoder.getPosition(), SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(elevatorRightEncoder.getPosition(), SparkMax.ControlType.kPosition);
  

//   public void setElevatorPositionUp(){
//     elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorUp, SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorUp, SparkMax.ControlType.kPosition);
//   }
//   public void setElevatorPositionDown(){
//     elevatorLeftPIDController.setReference(Constants.ElevatorPositions.leftElevatorDown, SparkMax.ControlType.kPosition);
//     elevatorRightPIDController.setReference(Constants.ElevatorPositions.rightElevatorDown, SparkMax.ControlType.kPosition);
//   }
//   

  /*Create manually controlled commands here */

   public Command intakePivotUpCommand(){
      return run(() -> intakePivotUp()).withTimeout(0.1);
    }

    public Command intakePivotDownCommand(){
      return run(() -> intakePivotDown()).withTimeout(0.1);
    }

    public Command intakePivotStopCommand(){
      return run(() -> intakePivotStop());
    }

    public Command setIntakePivotScoreL1()
    {
      return run(() -> intakePivotScoreL1()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScorel1) < 1));
    }
    public Command setIntakePivotScoreL2()
    {
      return run(() -> intakePivotScoreL2()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScorel2) < 1));
    }
    public Command setIntakePivotScoreL3()
    {
      return run(() -> intakePivotScoreL3()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScorel3) < 1));
    }
    public Command setIntakePivotScoreL4()
    {
      return run(() -> intakePivotScoreL4()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScorel4) < 1));
    }
    public Command setIntakePivotCoralPickup()
    {
      return run(() -> intakePivotCoralPickup()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotCoralPickup) < 1));
    }

    /*Create set position commands here */

//  public Command setElevatorPositionUpCommand(){
//     return run(() -> setElevatorPositionUp()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorUp) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorUp) < 1));
//   }
//   public Command setElevatorPositionDownCommand(){
//     return run(() -> setElevatorPositionDown()).until(() -> (Math.abs(elevatorRightEncoder.getPosition() - Constants.ElevatorPositions.rightElevatorDown) < 1) && (Math.abs(elevatorLeftEncoder.getPosition() - Constants.ElevatorPositions.leftElevatorDown) < 1));
//   }
  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePivotPosition",intakePivotEncoder.getPosition());
    }
  }

