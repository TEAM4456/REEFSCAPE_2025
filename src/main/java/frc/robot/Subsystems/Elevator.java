package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder; 
    private SparkClosedLoopController elevatorPIDController;
    private SparkMaxConfig elevatorConfig;

  public Elevator() {
    elevatorMotor = new SparkMax(18, MotorType.kBrushless);
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorConfig = new SparkMaxConfig();
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.closedLoop.pidf(1,0,0,0);
    elevatorConfig.openLoopRampRate(0.5);
    elevatorConfig.smartCurrentLimit(40);
   }
  
  /* Create your elevator Methods here */

  /*Manual Methods, button needs to be held for movement*/

  //Moves the motor at negative elevatorSpeed to go up
  public void elevatorUp(){
    elevatorMotor.set(-Constants.ElevatorPositions.elevatorSpeed);
  }
  //Moves the moter at positive elevatorSpeed to go down
  public void elevatorDown(){
    elevatorMotor.set(Constants.ElevatorPositions.elevatorSpeed);
  }
  //Stops the motor movment
   public void elevatorStop(){
     elevatorMotor.set(0);
  }
  
  /*Set Position Methods, button is pressed once for movement*/
   public void elevatorScoreL1(){
     elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorScoreL1, SparkBase.ControlType.kPosition);
   }

   public void elevatorScoreL2(){
     elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorScoreL2, SparkBase.ControlType.kPosition);
   }

    public void elevatorScoreL3(){
      elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorScoreL3, SparkBase.ControlType.kPosition);
    }

    public void elevatorScoreL4(){
      elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorScoreL4, SparkBase.ControlType.kPosition);
    }

    public void elevatorCoralPickupPosition(){
      elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorCoralPickupPosition, SparkBase.ControlType.kPosition);
    }

    public void elevatorClimbPosition(){
      elevatorPIDController.setReference(Constants.ElevatorPositions.ElevatorClimbPosition, SparkBase.ControlType.kPosition);
    }


  /*Create manually controlled commands here
   Refrence these in the robot run file
  */

   public Command elevatorUpCommand(){
      return run(() -> elevatorUp());
    }

    public Command elevatorDownCommand(){
      return run(() -> elevatorDown());
    }

    public Command elevatorStopCommand(){
      return run(() -> elevatorStop());
    }

    /*Create set position commands here */

    public Command elevatorScoreL1Command(){
      return run(() -> elevatorScoreL1()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorScoreL1) < 1));
    }
    public Command elevatorScoreL2Command(){
      return run(() -> elevatorScoreL2()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorScoreL2) < 1));
    }
    public Command elevatorScoreL3Command(){
      return run(() -> elevatorScoreL3()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorScoreL3) < 1));
    }
    public Command elevatorScoreL4Command(){
      return run(() -> elevatorScoreL4()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorScoreL4) < 1));
    }
    public Command elevatorCoralPickupPositionCommand(){
      return run(() -> elevatorCoralPickupPosition()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorCoralPickupPosition) < 1));
    }
    public Command elevatorClimbPositionCommand(){
      return run(() -> elevatorClimbPosition()).until(() -> (Math.abs(elevatorEncoder.getPosition() - Constants.ElevatorPositions.ElevatorClimbPosition) < 1));
    }


  @Override
    public void periodic(){
      SmartDashboard.putNumber("elevatorPosition",elevatorEncoder.getPosition());
    }
  }

