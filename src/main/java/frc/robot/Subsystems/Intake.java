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
    rightIntakeConfig.smartCurrentLimit(40);

    leftIntakeMotor = new SparkMax(20, MotorType.kBrushless);
    leftIntakeEncoder = rightIntakeMotor.getEncoder();
    leftIntakeConfig = new SparkMaxConfig();
    leftIntakeConfig.idleMode(IdleMode.kBrake);
    leftIntakeConfig.closedLoop.pidf(1,0,0,0);
    leftIntakeConfig.smartCurrentLimit(40);

   }
  
  /* Create your intake Methods here */

  //When called, this moves both motors to pickup coral
  public void intakePickupCoral(){
    rightIntakeMotor.set(-Constants.IntakeSpeeds.intakePickupCoral);
    leftIntakeMotor.set(Constants.IntakeSpeeds.intakePickupCoral);
  }
  //When called, this moves both motors to score coral
  public void intakeScoreCoral(){
    rightIntakeMotor.set(Constants.IntakeSpeeds.intakeScoreCoral);
    leftIntakeMotor.set(-Constants.IntakeSpeeds.intakeScoreCoral);
  }
   public void intakeStop(){
     rightIntakeMotor.set(0);
     leftIntakeMotor.set(0);
}


  /*Create manually controlled commands here */

   public Command intakePickupCoralCommand(){
      return run(() -> intakePickupCoral());
    }

    public Command intakeScoreCoralCommand(){
      return run(() -> intakeScoreCoral());
    }

    public Command intakeStopCommand(){
      return run(() -> intakeStop());
    }

    /*Create set position commands here */

  @Override
    public void periodic(){
      SmartDashboard.putNumber("intakePosition",rightIntakeEncoder.getPosition());
      SmartDashboard.putNumber("intakePosition",leftIntakeEncoder.getPosition());
    }
  }