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


public class AlgaePickup extends SubsystemBase {
    private SparkMax rightAlgaePickupMotor;
    private RelativeEncoder rightAlgaePickupEncoder; 
    private SparkClosedLoopController rightAlgaePickupPIDController;
    private SparkMaxConfig rightAlgaePickupConfig;

    private SparkMax leftAlgaePickupMotor;
    private RelativeEncoder leftAlgaePickupEncoder; 
    private SparkClosedLoopController leftAlgaePickupPIDController;
    private SparkMaxConfig leftAlgaePickupConfig;

  public AlgaePickup() {

    //Make a right and left intake for this subsystem, CanID 19 and 20
    rightAlgaePickupMotor = new SparkMax(20, MotorType.kBrushless);
    rightAlgaePickupEncoder = rightAlgaePickupMotor.getEncoder();
    rightAlgaePickupConfig = new SparkMaxConfig();
    rightAlgaePickupConfig.idleMode(IdleMode.kBrake);
    rightAlgaePickupConfig.closedLoop.pidf(1,0,0,0);
    rightAlgaePickupConfig.openLoopRampRate(0.5);
    rightAlgaePickupConfig.smartCurrentLimit(40);
    

    leftAlgaePickupMotor = new SparkMax(22, MotorType.kBrushless);
    leftAlgaePickupEncoder = leftAlgaePickupMotor.getEncoder();
    leftAlgaePickupConfig = new SparkMaxConfig();
    leftAlgaePickupConfig.idleMode(IdleMode.kBrake);
    leftAlgaePickupConfig.closedLoop.pidf(1,0,0,0);
    leftAlgaePickupConfig.openLoopRampRate(0.5);
    leftAlgaePickupConfig.smartCurrentLimit(40);
   }
  
  /* Create your intake Methods here */

  //When called, this moves both motors to pickup coral
  public void algaePickupIn(){
    rightAlgaePickupMotor.set(-Constants.AlgaePickupPositions.algaePickupSpeed);
    leftAlgaePickupMotor.set(Constants.AlgaePickupPositions.algaePickupSpeed);
  }
  public void algaePickupOut(){
    rightAlgaePickupMotor.set(Constants.AlgaePickupPositions.algaePickupSpeed);
    leftAlgaePickupMotor.set(-Constants.AlgaePickupPositions.algaePickupSpeed);
  }
   public void algaePickupStop(){
     rightAlgaePickupMotor.set(0);
     leftAlgaePickupMotor.set(0);
    
}


  /*Create manually controlled commands here */

    public Command algaePickupInCommand(){
        return run(() -> algaePickupIn());
    }

    public Command algaePickupOutCommand(){
        return run(() -> algaePickupOut());
    }

    public Command algaePickupStopCommand(){
      return run(() -> algaePickupStop());
    }

    /*Create set position commands here */

  @Override
    public void periodic(){
      SmartDashboard.putNumber("rightAlgaePickupPosition",rightAlgaePickupEncoder.getPosition());
      SmartDashboard.putNumber("leftAlgaePickupPosition",leftAlgaePickupEncoder.getPosition());
    }
  }
