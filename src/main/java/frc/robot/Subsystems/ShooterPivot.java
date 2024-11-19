package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ShooterPivot extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private RelativeEncoder pivotMotorEncoder;
    private final SparkPIDController pivotMotorPIDController;

  public ShooterPivot() {
    pivotMotor = new CANSparkMax(20,MotorType.kBrushless);
    pivotMotor.setOpenLoopRampRate(.5);
    pivotMotorEncoder = pivotMotor.getEncoder();
    pivotMotorPIDController = pivotMotor.getPIDController();

   

    pivotMotorPIDController.setP(.5);
    pivotMotorPIDController.setI(0);
    pivotMotorPIDController.setD(0);
    pivotMotorPIDController.setFF(0);

  

  }
    
  public void ShooterPivotUp(){
    pivotMotor.set(Constants.ShooterPivotPositions.shooterPivotSpeed);
    pivotMotor.set(Constants.ShooterPivotPositions.shooterPivotSpeed);
  }
  public void ShooterPivotDown(){
    pivotMotor.set(-Constants.ShooterPivotPositions.shooterPivotSpeed);
    pivotMotor.set(-Constants.ShooterPivotPositions.shooterPivotSpeed);
  }
  public void ShooterPivotStop(){
    pivotMotor.set(0);
    pivotMotor.set(0);
  }
  public RelativeEncoder getPivotEncoder(){
    return pivotMotorEncoder;
  }
  public void shooterPositionDown(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionDown, CANSparkMax.ControlType.kPosition);
  }
  public void shooterPositionCorner(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionShootSide, CANSparkMax.ControlType.kPosition);
  }
   public void shooterPositionCenter(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionShootCenter, CANSparkMax.ControlType.kPosition);
   }
  public void shooterPositionUp(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionUp+2, CANSparkMax.ControlType.kPosition);
  }
  public void shooterPositionSource(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionSource, CANSparkMax.ControlType.kPosition);
  }
  public void shooterPositionAmp(){
    pivotMotorPIDController.setReference(Constants.ShooterPivotPositions.shooterPositionAmp, CANSparkMax.ControlType.kPosition);
  }

  
  public Command shooterPositionDownCommand(){
    return run(() -> shooterPositionDown()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionDown) < 1));
  }
  public Command shooterPositionUpCommand(){
    return run(() -> shooterPositionUp()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionUp) < 1));
  }
  public Command shooterPositionCenterCommand(){
    return run(() -> shooterPositionCenter()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionShootCenter) < 1));
  }
  public Command shooterPositionCornerCommand(){
    return run(() -> shooterPositionCorner()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionShootSide) < 1));
  }
  public Command shooterPositionSourceCommand(){
    return run(() -> shooterPositionSource()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionSource) < 1));
  }
  public Command shooterPositionAmpCommand(){
    return run(() -> shooterPositionAmp()).until(() -> (Math.abs(pivotMotorEncoder.getPosition() - Constants.ShooterPivotPositions.shooterPositionAmp) < 1));
  }
 







  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Pivot Position",pivotMotorEncoder.getPosition());
}
}
