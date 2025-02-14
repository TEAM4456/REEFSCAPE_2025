
// Was used last year for the climb onto chain
// and could could potentially be used for the deep cage...Maybe

package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
        leftClimberConfig.closedLoop.pidf(1,0,0,0);
        leftClimberConfig.openLoopRampRate(0.5);
        leftClimberConfig.smartCurrentLimit(40);

        rightClimberMotor = new SparkMax(17, MotorType.kBrushless); // sets up right climber motor
        rightClimberEncoder =  rightClimberMotor.getEncoder(); // sets up right climber motor encoder
        rightClimberConfig = new SparkMaxConfig(); // CONFIGURATIONS FOR RIGHT CLIMBER MOTOR BELOW
        rightClimberConfig.idleMode(IdleMode.kBrake);
        rightClimberConfig.closedLoop.pidf(1,0,0,0);
        rightClimberConfig.openLoopRampRate(0.5);
        rightClimberConfig.smartCurrentLimit(40);
    }
    //Create your climber Methods here

    /*Manual Methods */
        
        public void climberUp() {
            leftClimberMotor.set(-Constants.ClimberPositions.climberSpeed);
            rightClimberMotor.set(Constants.ClimberPositions.climberSpeed);
        }
        public void climberDown() {
            leftClimberMotor.set(Constants.ClimberPositions.climberSpeed);
            rightClimberMotor.set(-Constants.ClimberPositions.climberSpeed);
        }

        public void climberStop() {
            leftClimberMotor.set(0);
            rightClimberMotor.set(0);
        }

    /*Set Position Methods */

        public void ClimbDeepCage() {
            leftClimberPIDController.setReference(Constants.ClimberPositions.ClimbDeepCageLeft, SparkBase.ControlType.kPosition);
            rightClimberPIDController.setReference(Constants.ClimberPositions.ClimbDeepCageRight, SparkBase.ControlType.kPosition);
        }

    /*Create manually controlled commands here */
    
    //TODO: Create climberUpCommand and climberDownCommand
    
    public Command climberStopCommand(){
      return run(() -> climberStop());
    }



    /*Create set position commands here */
    //TODO: Create command for climber to go to a set position called ClimbDeepCageCommand

    }
