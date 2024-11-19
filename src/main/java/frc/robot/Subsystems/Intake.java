package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private RelativeEncoder intakeEncoder;
    private final SparkPIDController intakePIDController;


    private CANSparkMax intakeFeed;
    private RelativeEncoder intakeFeedEncoder;
    private final SparkPIDController intakeFeedPIDController;

  public Intake() {
    intake = new CANSparkMax(16,MotorType.kBrushless);
    intakeEncoder = intake.getEncoder();
    intakePIDController = intake.getPIDController();

    intakeFeed = new CANSparkMax(22,MotorType.kBrushless);
    intakeFeedEncoder = intake.getEncoder();
    intakeFeedPIDController = intake.getPIDController();


  }
  public void intakeFeedForward(){
    intakeFeed.set(-Constants.IntakeConstants.intakeFeedSpeed);
  }
  public void intakeFeedBack(){
    intakeFeed.set(Constants.IntakeConstants.intakeFeedSpeed);
  }
  public void intakeFeedStop(){
    intakeFeed.set(0);
  }
  public void speedForward(){
    intake.set(Constants.IntakeConstants.intakeSpeed);
  }
  public void speedBack(){
    intake.set(-Constants.IntakeConstants.intakeSpeed);
  }
  public void speedStop(){
    intake.set(0);
  }
  @Override
  public void periodic() {

}
}
