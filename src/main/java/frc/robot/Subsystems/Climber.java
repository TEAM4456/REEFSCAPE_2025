
// Was used last year for the climb onto chain
// and could could potentially be used for the deep cage...Maybe

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    // p - proportional gain value i,- integral gain value,d - derivative gain value
    private SparkMax leftClimberMotor;
    private RelativeEncoder leftClimberEncoder;
    private SparkClosedLoopController leftClimberPIDController;
    private SparkMaxConfig leftClimberConfig;

    private SparkMax rightClimberMotor;
    private RelativeEncoder rightClimberEncoder;
    private SparkClosedLoopController rightClimberPIDController;
    private SparkMaxConfig rightClimberConfig;

    public Climber() {
        leftClimberMotor  = new SparkMax(16, MotorType.kBrushless); // sets up left climber motor
        leftClimberEncoder =  leftClimberMotor.getEncoder(); // sets up left climber motor encoder 
        leftClimberConfig = new SparkMaxConfig();  // CONFIGURATIONS FOR LEFT CLIMBER MOTOR BELOW
        leftClimberConfig.idleMode(IdleMode.kBrake);
        leftClimberConfig.closedLoop.pid(1,0,0);

        rightClimberMotor = new SparkMax(17, MotorType.kBrushless); // sets up right climber motor
        rightClimberEncoder =  rightClimberMotor.getEncoder(); // sets up right climber motor encoder
        rightClimberConfig = new SparkMaxConfig(); // CONFIGURATIONS FOR RIGHT CLIMBER MOTOR BELOW
        rightClimberConfig.idleMode(IdleMode.kBrake);
        rightClimberConfig.closedLoop.pid(1,0,0);
        /*
        public void climberUp() {
            leftClimberMotor.set(-Constants.ClimberPositions.climberSpeed);
            rightClimberMotor.set(Constants.ClimberPositions.climberSpeed);
        }
        public void climberDown() {
            leftClimberMotor.set(-Constants.ClimberPositions.climberSpeed);
            rightClimberMotor.set(Constants.ClimberPositions.climberSpeed);
        }
        */
    }
}
