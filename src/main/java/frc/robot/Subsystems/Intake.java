package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    //Make a right and left intake for this subsystem, CanID 19 and 20
    intakeMotor = new SparkMax(19, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    intakeConfig = new SparkMaxConfig();
    intakeConfig.idleMode(IdleMode.kBrake);
    intakeConfig.closedLoop.pidf(1,0,0,0);
    intakeConfig.smartCurrentLimit(40);

   }
  
  /* Create your intake Methods here */

  //When called, this moves both motors to pickup coral
  public void intakePickupCoral(){
    intakeMotor.set(-Constants.IntakeSpeeds.intakePickupCoral);
  }
  public void intakeAutoPullBack(){
    intakePIDController.setReference(Constants.IntakeSpeeds.intakePullBack, SparkMax.ControlType.kPosition);
  }
  //When called, this moves both motors to score coral
  public void intakeScoreCoralL2toL4(){
    intakeMotor.set(Constants.IntakeSpeeds.intakeScoreCoralL2toL4);
  }
  public void intakeScoreCoralL1(){
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

    public Command intakeScoreCoralL2toL4Command(){
      return run(() -> intakeScoreCoralL2toL4());
    }

    public Command intakeScoreCoralL1Command(){
      return run(() -> intakeScoreCoralL1());
    }
    public Command intakeRemoveAlgaeCommand(){
      return run(() -> intakeRemoveAlgae());
    }

    public Command intakeStopCommand(){
      return run(() -> intakeStop());
    }

    /*Create set position commands here */

  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePosition",intakeEncoder.getPosition());
    }
  }