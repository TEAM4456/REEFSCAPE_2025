package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
    private CANSparkMax shooter;
    private RelativeEncoder shooterEncoder;
    private final SparkPIDController shooterPIDController;

    private CANSparkMax feeder;
    private RelativeEncoder feederEncoder;
    private final SparkPIDController feederPIDController;

    private CANSparkMax hatch;
    private RelativeEncoder hatchEncoder;
    private final SparkPIDController hatchPIDController;

  public Shooter() {
    shooter = new CANSparkMax(17,MotorType.kBrushless);
    shooterEncoder = shooter.getEncoder();
    shooterPIDController = shooter.getPIDController();

    feeder = new CANSparkMax(18,MotorType.kBrushless);
    feederEncoder = feeder.getEncoder();
    feederPIDController = feeder.getPIDController();

    hatch = new CANSparkMax(21,MotorType.kBrushless);
    hatchEncoder = hatch.getEncoder();
    hatchPIDController = hatch.getPIDController();

    hatchPIDController.setP(.1);



  }
  public void shooterOn(){
    shooter.set(-Constants.ShooterConstants.shootSpeed);
  }
  public void shooterOff(){
    shooter.set(0);
  }
  public void shooterIntake(){
    shooter.set(Constants.ShooterConstants.intakeSpeed);
  }
  public void shooterAmp(){
    shooter.set(-Constants.ShooterConstants.ampSpeed);
  }
  public void shooterHatch(){
    shooter.set(-Constants.ShooterConstants.hatchSpeed);
  }
  public void feedForward(){
    feeder.set(-Constants.ShooterConstants.feedSpeed);
  }
   public void feedForwardSlow(){
    feeder.set(-Constants.ShooterConstants.feedSpeed/5);
  }
  public void feedStop(){
    feeder.set(0);
  }
  public void feedBack(){
    feeder.set(Constants.ShooterConstants.feedSpeed/5);
  }

  public void hatchForward(){
    hatch.set(.15);
  }
  public void hatchBack(){
    hatch.set(-.15);
  }
  public void hatchStop(){
    hatch.set(0);
  }
  
  public void hatchPositionOpen(){
    hatchPIDController.setReference(Constants.ShooterConstants.hatchOpen, CANSparkMax.ControlType.kPosition);
  }
  public void hatchPositionClose(){
    hatchPIDController.setReference(Constants.ShooterConstants.hatchClose, CANSparkMax.ControlType.kPosition);
  }
  public void hatchPositionAmp(){
    hatchPIDController.setReference(Constants.ShooterConstants.hatchAmp, CANSparkMax.ControlType.kPosition);
  }

  public Command hatchPositionOpenCommand(){
    return run(() -> hatchForward()).until(() -> (Math.abs(hatchEncoder.getPosition() - Constants.ShooterConstants.hatchOpen) < 1));
  }
  public Command hatchPositionAmpCommand(){
    return run(() -> hatchPositionAmp()).until(() -> (Math.abs(hatchEncoder.getPosition() - Constants.ShooterConstants.hatchAmp) < 1));
  }
  public Command hatchPositionCloseCommand(){
    return run(() -> hatchPositionClose()).until(() -> (Math.abs(hatchEncoder.getPosition() - Constants.ShooterConstants.hatchClose) < 1));
  }
  


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hatch Encoder",hatchEncoder.getPosition());
}
}
