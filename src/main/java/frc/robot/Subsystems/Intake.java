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


public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;
    private RelativeEncoder intakeEncoder; 
    private SparkClosedLoopController intakePIDController;
    private SparkMaxConfig intakeConfig;

  public Intake() {
    intakeMotor = new SparkMax(18, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop.pid(1,0,0);

    /*  // look into getting rid of this? Not sure if needed? Sat 1/25
   //intakeMotor.setOpenLoopRampRate(.5);
    intakeEncoder = intakeMotor.getEncoder();
    intakePIDController = intakeMotor.getClosedLoopController();

     //intakePIDController.pid(1, 0, 0);
     intakePIDController.setFF(0); */
   }
  
  /* Create your intake Methods (? correct terminology) here */

  //When called, this moves the motor at negative intakeSpeed to go up
  public void intakeUp(){
    intakeMotor.set(-Constants.IntakePositions.intakeSpeed);
  }
  //When called, this moves the moter at positive elevatorSpeed to go down
  public void intakeDown(){
    intakeMotor.set(Constants.IntakePositions.intakeSpeed);
  }
   public void intakeStop(){
     intakeMotor.set(0);
}
//     intakeLeftPIDController.setReference(intakeLeftEncoder.getPosition(), SparkMax.ControlType.kPosition);
//     intakeRightPIDController.setReference(intakeRightEncoder.getPosition(), SparkMax.ControlType.kPosition);
  

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

   public Command intakeUpCommand(){
      return run(() -> intakeUp()).withTimeout(0.1);
    }

    public Command intakeDownCommand(){
      return run(() -> intakeDown()).withTimeout(0.1);
    }

    public Command intakeStopCommand(){
      return run(() -> intakeStop());
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
      SmartDashboard.putNumber("intakePosition",intakeEncoder.getPosition());
    }
  }