/*
Below is copied exactly from elevator.java
*/
package frc.robot.Subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
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
import frc.robot.Constants.IntakePivotPositions;

public class IntakePivot extends SubsystemBase {
    private SparkMax intakePivotMotor;
    private RelativeEncoder intakePivotEncoder; 
    private SparkClosedLoopController intakePivotPIDController;
    private SparkMaxConfig intakePivotConfig;

    public IntakePivot() {
        intakePivotMotor = new SparkMax(21, MotorType.kBrushless);
        intakePivotEncoder = intakePivotMotor.getEncoder();
        intakePivotConfig = new SparkMaxConfig();
        intakePivotConfig.idleMode(IdleMode.kBrake);
        intakePivotConfig.closedLoop.pidf(1,0,0,0);
        intakePivotConfig.openLoopRampRate(0.5);
        intakePivotConfig.smartCurrentLimit(40);
}
  
  /* Create your intakePivot Methods here */

  /*Manual Methods*/
  //When called, this moves the motor at negative elevatorSpeed to go up
  public void intakePivotUp(){
    intakePivotMotor.set(-Constants.IntakePivotPositions.intakePivotSpeed);
  }
  //When called, this moves the moter at positive elevatorSpeed to go down
  public void intakePivotDown(){
    intakePivotMotor.set(Constants.IntakePivotPositions.intakePivotSpeed);
  }
   public void intakePivotStop(){
    intakePivotMotor.set(0);
}

  /*Set Position Methods*/
  public void intakePivotScoreL1(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL1, SparkBase.ControlType.kPosition);
  }
  
  public void intakePivotScoreL2(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL2, SparkBase.ControlType.kPosition);
  }

  public void intakePivotScoreL3(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL3, SparkBase.ControlType.kPosition);
  }
  
  public void intakePivotScoreL4(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL4, SparkBase.ControlType.kPosition);
  }

  public void intakePivotCoralPickupPosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotCoralPickupPosition, SparkBase.ControlType.kPosition);
  }
  public void intakePivotClimbPosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotClimbPosition, SparkBase.ControlType.kPosition);
  }
  


  /*Create manually controlled commands here */

   public Command intakePivotUpCommand(){
      return run(() -> intakePivotUp());
    }

    public Command intakePivotDownCommand(){
      return run(() -> intakePivotDown());
    }

    public Command intakePivotStopCommand(){
      return run(() -> intakePivotStop());
    }

    /*Create set position commands here */
    public Command IntakePivotScoreL1Command(){
      return run(() -> intakePivotScoreL1()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL1) < 1));
    }
    
    public Command IntakePivotScoreL2Command(){
      return run(() -> intakePivotScoreL2()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL2) < 1));
    }
    
    public Command IntakePivotScoreL3Command(){
      return run(() -> intakePivotScoreL3()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL3) < 1));
    }
    
    public Command IntakePivotScoreL4Command(){
      return run(() -> intakePivotScoreL4()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL4) < 1));
    }
   
    public Command IntakePivotCoralPickupPositionCommand(){
      return run(() -> intakePivotCoralPickupPosition()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotCoralPickupPosition) < 1));
    }

    public Command IntakePivotClimbPositionCommand(){
      return run(() -> intakePivotClimbPosition()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotClimbPosition) < 1));
    }

    
  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePivotPosition",intakePivotEncoder.getPosition());
    }
  }

