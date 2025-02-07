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
        intakePivotMotor = new SparkMax(21, MotorType.kBrushless);
        intakePivotEncoder = intakePivotMotor.getEncoder();
        intakePivotConfig = new SparkMaxConfig();
        intakePivotConfig.idleMode(IdleMode.kBrake);
        intakePivotConfig.closedLoop.pidf(1,0,0,0);
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
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL1, SparkMax.ControlType.kPosition);
  }
  
  public void intakePivotScoreL2(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL2, SparkMax.ControlType.kPosition);
  }

  public void intakePivotScoreL3(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL3, SparkMax.ControlType.kPosition);
  }
  
  public void intakePivotScoreL4(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotScoreL4, SparkMax.ControlType.kPosition);
  }

  public void intakePivotCoralPickupPosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotCoralPickupPosition, SparkMax.ControlType.kPosition);
  }
  


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

    /*Create set position commands here */
    public Command setIntakePivotScoreL1(){
      return run(() -> intakePivotScoreL1()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL1) < 1));
    }
    
    public Command setIntakePivotScoreL2(){
      return run(() -> intakePivotScoreL2()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL2) < 1));
    }
    
    public Command setIntakePivotScoreL3(){
      return run(() -> intakePivotScoreL3()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL3) < 1));
    }
    
    public Command setIntakePivotScoreL4(){
      return run(() -> intakePivotScoreL4()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL4) < 1));
    }
   
    public Command setIntakePivotCoralPickup(){
      return run(() -> intakePivotCoralPickupPosition()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotCoralPickupPosition) < 1));
    }

    ;
//   }
  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePivotPosition",intakePivotEncoder.getPosition());
    }
  }

