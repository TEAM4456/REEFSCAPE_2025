package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakePulley extends SubsystemBase {
    private CANSparkMax pulleyMotor;
    private RelativeEncoder pulleyMotorEncoder;
    private final SparkPIDController pulleyMotorPIDController;

  public IntakePulley() {
    pulleyMotor = new CANSparkMax(19,MotorType.kBrushless);
    pulleyMotor.setOpenLoopRampRate(.5);
    pulleyMotorEncoder = pulleyMotor.getEncoder();
    pulleyMotorPIDController = pulleyMotor.getPIDController();

   

    pulleyMotorPIDController.setP(.1);
    pulleyMotorPIDController.setI(0);
    pulleyMotorPIDController.setD(0);
    pulleyMotorPIDController.setFF(0);

  

  }
    
  public void moveIntakeIn(){
    pulleyMotor.set(Constants.IntakeConstants.pulleySpeed);
  }
  public void moveIntakeOut(){
    pulleyMotor.set(-Constants.IntakeConstants.pulleySpeed);
  }
  public void moveIntakeStop(){
    pulleyMotor.set(0);
  }

  public void intakePositionGround(){
    pulleyMotorPIDController.setReference(Constants.IntakeConstants.intakePositionGround, CANSparkMax.ControlType.kPosition);
  }
  public void intakePositionFeed(){
    pulleyMotorPIDController.setReference(Constants.IntakeConstants.intakePositionFeed, CANSparkMax.ControlType.kPosition);
  }
  public void intakePositionClimb(){
    pulleyMotorPIDController.setReference(Constants.IntakeConstants.intakePositionClimb, CANSparkMax.ControlType.kPosition);
  }

  public Command intakePositionClimbCommand(){
    return run(() -> intakePositionClimb()).until(() -> (Math.abs(pulleyMotorEncoder.getPosition() - Constants.IntakeConstants.intakePositionClimb) < 1));
  }
  public Command intakePositionGroundCommand(){
    return run(() -> intakePositionGround()).until(() -> (Math.abs(pulleyMotorEncoder.getPosition() - Constants.IntakeConstants.intakePositionGround) < 1));
  }
  public Command intakePositionFeedCommand(){
    return run(() -> intakePositionFeed()).until(() -> (Math.abs(pulleyMotorEncoder.getPosition() - Constants.IntakeConstants.intakePositionFeed) < 1));
  }




  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake pulley Position",pulleyMotorEncoder.getPosition());
}
}