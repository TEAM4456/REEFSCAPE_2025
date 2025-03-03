//Below is copied exactly from elevator.java

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

  /*Manual Methods, these are the ones where you hold a button to make it go up or down*/
  public void intakePivotUp(){
    intakePivotMotor.set(-Constants.IntakePivotPositions.intakePivotSpeed);
  }
  public void intakePivotDown(){
    intakePivotMotor.set(Constants.IntakePivotPositions.intakePivotSpeed);
  }
   public void intakePivotStop(){
    intakePivotMotor.set(0);
}

  /*Set Position Methods, these are the ones where you press a button once and it automatically goes to the wanted position*/
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
  public void intakePivotAlgaeHigh(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotAlgaeHigh, SparkBase.ControlType.kPosition);
  }
  public void intakePivotAlgaeLow(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotAlgaeLow, SparkBase.ControlType.kPosition);
  }
  public void intakePivotCoralPickupPosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotCoralPickupPosition, SparkBase.ControlType.kPosition);
  }
  public void intakePivotClimbPosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotClimbPosition, SparkBase.ControlType.kPosition);
  }
  public void intakePivotDrivePosition(){
    intakePivotPIDController.setReference(Constants.IntakePivotPositions.intakePivotDrivePosition, SparkBase.ControlType.kPosition);
  }
  

  //Create manually controlled commands here, these are the commands 
  //that are called from RobotContainer.java (or maybe just Robot.java) to move the motors 

   public Command intakePivotUpCommand(){
      return run(() -> intakePivotUp());
    }

    public Command intakePivotDownCommand(){
      return run(() -> intakePivotDown());
    }

    public Command intakePivotStopCommand(){
      return run(() -> intakePivotStop());
    }

    public Command intakePivotScoreL1Command(){
      return run(() -> intakePivotScoreL1()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL1) < 1));
    }
    
    public Command intakePivotScoreL2Command(){
      return run(() -> intakePivotScoreL2()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL2) < 1));
    }
    
    public Command intakePivotScoreL3Command(){
      return run(() -> intakePivotScoreL3()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL3) < 1));
    }
    
    public Command intakePivotScoreL4Command(){
      return run(() -> intakePivotScoreL4()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotScoreL4) < 1));
    }

    public Command intakePivotAlgaeHighCommand(){
      return run(() -> intakePivotAlgaeHigh()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotAlgaeHigh) < 1));
    }

    public Command intakePivotAlgaeLowCommand(){
      return run(() -> intakePivotAlgaeLow()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotAlgaeLow) < 1));
    }
   
    public Command intakePivotCoralPickupPositionCommand(){
      return run(() -> intakePivotCoralPickupPosition()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotCoralPickupPosition) < 1));
    }

    public Command intakePivotClimbPositionCommand(){
      return run(() -> intakePivotClimbPosition()).until(() -> (Math.abs(intakePivotEncoder.getPosition() - Constants.IntakePivotPositions.intakePivotClimbPosition) < 1));
    }
    
  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePivotPosition",intakePivotEncoder.getPosition());
    }
  }

