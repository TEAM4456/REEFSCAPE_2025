package frc.robot.Subsystems;

// Imports for the liear actuator
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;

// Imports for the command system
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports for the constants
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {

    //Object declarations
    private SparkMax pivotElvMotor;
    private RelativeEncoder pivotElvMotorEncoder;
    private final ClosedLoopConfig pivotElvMotorController;

    public ElevatorPivot() {
        //Setting the object's values
        pivotElvMotor = new SparkMax(15, MotorType.kBrushless);
        pivotElvMotorEncoder = pivotElvMotor.getEncoder();
        pivotElvMotorController = new ClosedLoopConfig();

        //Setting the PID values
        pivotElvMotorController.pid(Constants.Swerve.angleKP, Constants.Swerve.angleKI, Constants.Swerve.angleKD);
    }

}