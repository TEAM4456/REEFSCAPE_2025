package frc.robot.Subsystems;

// Imports for the liear actuator
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
;

// Imports for the command system
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports for the constants
import frc.robot.Constants;

public class ElevatorPivot extends SubsystemBase {

    private SparkMax pivotElvMotor;
    private RelativeEncoder pivotElvMotorEncoder;
    private final ClosedLoopConfig pivotElvMotorController;

    public ElevatorPivot() {
        pivotElvMotor = new SparkMax(15, MotorType.kBrushless);
        pivotElvMotorEncoder = pivotElvMotor.getEncoder();
        pivotElvMotorController = pivotElvMotor.getPIDController();
    }

    

}
