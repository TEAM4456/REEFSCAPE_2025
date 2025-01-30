package frc.robot.Subsystems;


// Imports for the linear actuator


import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


// Imports for the command system
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports for the constants
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {


    //Object declarations
    private SparkMax pivotElvMotor;
    private SparkMaxConfig pivotElvConfig;
    private RelativeEncoder pivotElvMotorEncoder;
    private final ClosedLoopConfig pivotElvPIDController;

    public ElevatorPivot() {
        //Setting the object's values
        pivotElvMotor = new SparkMax(15, MotorType.kBrushless);
        pivotElvMotorEncoder = pivotElvMotor.getEncoder();
        pivotElvPIDController = new ClosedLoopConfig();

        pivotElvConfig = new SparkMaxConfig();  //CONFIGURATIONS FOR Pivot Elevator MOTOR BELOW
        pivotElvConfig.idleMode(IdleMode.kBrake);
        pivotElvConfig.closedLoop.pid(1,0,0);

        
    }

   

}

