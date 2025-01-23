package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extend SubsystemBase {
    // p - proportional gain value i,- integral gain value,d - derivative gain value
    private SparkMax leftClimberMotor;
    private SparkMaxConfig leftClimberEncoder;
    private SparkClosedLoopController leftClimberPIDController;

    private SparkMax rightClimberMotor;
    private SparkMaxConfig rightClimberEncoder;
    private SparkClosedLoopController rightClimberPIDController;

    public Climber()
    {
        leftClimberMotor  = new SparkMax(16, MotorType.kBrushless);
        leftClimberEncoder = leftClimberMotor.getEncoder();
        leftClimberPIDController = leftClimberMotor.getSparkClosedLoopController();

        rightClimberMotor = new SparkMax(17, MotorType.kBrushless);
        rightClimberEncoder = rightClimberMotor.getEncoder();
        elevatorRightPIDController = rightClimberMotor.getSparkClosedLoopController();
    }
}
