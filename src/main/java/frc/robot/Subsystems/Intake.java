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
    private SparkMax rightIntakeMotor;
    private RelativeEncoder rightIntakeEncoder; 
    private SparkClosedLoopController rightIntakePIDController;
    private SparkMaxConfig rightIntakeConfig;

    private SparkMax leftIntakeMotor;
    private RelativeEncoder leftIntakeEncoder; 
    private SparkClosedLoopController leftIntakePIDController;
    private SparkMaxConfig leftIntakeConfig;

  public Intake() {

    //Make a right and left intake for this subsystem, CanID 19 and 20
    rightIntakeMotor = new SparkMax(19, MotorType.kBrushless);
    rightIntakeEncoder = rightIntakeMotor.getEncoder();
    rightIntakeConfig = new SparkMaxConfig();
    rightIntakeConfig.idleMode(IdleMode.kBrake);
    rightIntakeConfig.closedLoop.pidf(1,0,0,0);

    leftIntakeMotor = new SparkMax(19, MotorType.kBrushless);
    leftIntakeEncoder = rightIntakeMotor.getEncoder();
    leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IdleMode.kBrake);
    leftIntakeConfig.closedLoop.pidf(1,0,0,0);

    /*  // look into getting rid of this? Not sure if needed? Sat 1/25
   //intakeMotor.setOpenLoopRampRate(.5);
    intakeEncoder = intakeMotor.getEncoder();
    intakePIDController = intakeMotor.getClosedLoopController();

     //intakePIDController.pid(1, 0, 0);
     intakePIDController.setFF(0); */
   }
  
  /* Create your intake Methods here */

  //When called, this moves right motor at negative intakeSpeed to go up
  public void rightIntakePickupCoral(){
    rightIntakeMotor.set(-Constants.IntakeSpeeds.intakePickupCoral);
  }
  //When called, this moves right motor at positive elevatorSpeed to go down
  public void rightIntakeScoreCoral(){
    rightIntakeMotor.set(Constants.IntakeSpeeds.intakeScoreCoral);
  }
   public void rightIntakeStop(){
     rightIntakeMotor.set(0);
}
 //When called, this moves left motor at intakeSpeed to go up
 public void leftIntakePickupCoral(){
  leftIntakeMotor.set(Constants.IntakeSpeeds.intakePickupCoral);
}
//When called, this moves left motor at positive elevatorSpeed to go down
public void leftIntakeScoreCoral(){
  leftIntakeMotor.set(Constants.IntakeSpeeds.intakeScoreCoral);
}
 public void leftIntakeStop(){
   leftIntakeMotor.set(0);
}

  /*Create manually controlled commands here */

   public Command rightIntakePickupCoralCommand(){
      return run(() -> rightIntakePickupCoral());
    }

    public Command rightIntakeScoreCoralCommand(){
      return run(() -> rightIntakeScoreCoral());
    }

    public Command rightIntakeStopCommand(){
      return run(() -> rightIntakeStop());
    }



    public Command leftIntakePickupCoralCommand(){
      return run(() -> leftIntakePickupCoral());
    }

    public Command leftIntakeScoreCoralCommand(){
      return run(() -> leftIntakeScoreCoral());
    }

    public Command leftIntakeStopCommand(){
      return run(() -> leftIntakeStop());
    }

    /*Create set position commands here */

  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePosition",rightIntakeEncoder.getPosition());
      SmartDashboard.putNumber("intakePosition",leftIntakeEncoder.getPosition());
    }
  }