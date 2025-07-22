package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSpeeds;


public class Intake extends SubsystemBase {
    private SparkMax intakeMotor;
    private RelativeEncoder intakeEncoder; 
    private SparkClosedLoopController intakePIDController;
    private SparkMaxConfig intakeConfig;


  public Intake() {

    //Make a right and left intake for this subsystem, CanID 19 and 20
    intakeMotor = new SparkMax(19, MotorType.kBrushless);
    intakePIDController = intakeMotor.getClosedLoopController();
    intakeEncoder = intakeMotor.getEncoder();
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop.pidf(1,0,0,0);
    intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakeConfig.openLoopRampRate(0.5);
    intakeConfig.smartCurrentLimit(40);
    intakeMotor.configure(intakeConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }
  
  /* Create your intake Methods here */

  //When called, this moves both motors to pickup coral
  public void intakePickupCoral(){
    intakeMotor.set(-Constants.IntakeSpeeds.intakePickupCoral);
  }
  public void intakeAutoPullBack(){
    intakePIDController.setReference(Constants.IntakeSpeeds.intakePullBack, SparkBase.ControlType.kPosition);
  }
  public void intakeReset(){
    intakePIDController.setReference(0, SparkBase.ControlType.kPosition);
  }
  public void intakeAutoScoreL4(){
    intakePIDController.setReference(Constants.IntakeSpeeds.intakeAutoScoreL4, SparkBase.ControlType.kPosition);
  }
  public void intakeAutoScoreL1(){
    intakePIDController.setReference(Constants.IntakeSpeeds.intakeAutoScoreL1, SparkBase.ControlType.kPosition);
  }
  //When called, this moves both motors to score coral
  public void intakeScoreCoralL4(){
    intakeMotor.set(-Constants.IntakeSpeeds.intakeScoreCoralL2toL4);
  }
  public void intakeScoreCoralL2and3(){
    intakeMotor.set(Constants.IntakeSpeeds.intakeScoreCoralL1);
  }
  public void intakeRemoveAlgae(){
    intakeMotor.set(-Constants.IntakeSpeeds.intakeRemoveAlgae);
  }
   public void intakeStop(){
     intakeMotor.set(0);
    
}


  /*Create manually controlled commands here */

   public Command intakePickupCoralCommand(){
      return run(() -> intakePickupCoral());
    }

    public Command intakeAutoScoreL4Command(){
      return run(() -> intakeAutoScoreL4()).until(() -> (Math.abs(intakeEncoder.getPosition() - Constants.IntakeSpeeds.intakeAutoScoreL4) < 1));
    }

    public Command intakeAutoScoreL1Command(){
      return run(() -> intakeAutoScoreL1()).until(() -> (Math.abs(intakeEncoder.getPosition() - Constants.IntakeSpeeds.intakeAutoScoreL1) < 1));
    }

    public Command intakeScoreCoralL4Command(){
      return run(() -> intakeScoreCoralL4());
    }

    public Command intakeScoreCoralL2and3Command(){
      return run(() -> intakeScoreCoralL2and3());
    }
    public Command intakeRemoveAlgaeCommand(){
      return run(() -> intakeRemoveAlgae());
    }

    public Command intakeStopCommand(){
      return run(() -> intakeStop());
    }
    public Command intakeResetCommand(){
      return run(() -> intakeReset());
    }
    

    /*Create set position commands here */

    public Command intakeAutoPullBackCommand(){
      return run(() -> intakeAutoPullBack()).until(() -> (Math.abs(intakeEncoder.getPosition() - Constants.IntakeSpeeds.intakePullBack) < 1));
    }

  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePosition",intakeEncoder.getPosition());
    }
  }