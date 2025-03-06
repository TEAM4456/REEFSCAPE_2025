/* Was used last year for the climb onto chain
Could potentially be used for the deep cage...Maybe */

// Imports are declared here and allow new commands to be used
package frc.robot.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Declares the main body of the class and the stuff that happens within
public class Climber extends SubsystemBase {
    

    private SparkMax leftClimberMotor;
    private RelativeEncoder leftClimberEncoder;
    private SparkClosedLoopController leftClimberPIDController;
    private SparkMaxConfig leftClimberConfig; 

    // 1st constructor declaration area - 1st and 2nd are two parts for the same thing
    private SparkMax rightClimberMotor;
    private RelativeEncoder rightClimberEncoder;
    private SparkClosedLoopController rightClimberPIDController;
    private SparkMaxConfig rightClimberConfig;

    public Climber() {
        // 2nd constructor declaration area

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

    // Create your Manual and Set Position methods below

        // Manual Methods - buttons are held and realeased to create movement (ps, left is commented out until needed use)

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
        public void climbDrivePosition(){
            leftClimberPIDController.setReference(Constants.ClimberPositions.ClimbDriveLeft, SparkBase.ControlType.kPosition);
            rightClimberPIDController.setReference(Constants.ClimberPositions.ClimbDriveRight, SparkBase.ControlType.kPosition);
        }
    
        // Set Position Methods - buttons are presed once then automatically go to their positions

        public void ClimbDeepCage() {
            leftClimberPIDController.setReference(Constants.ClimberPositions.ClimbDeepCageLeft, SparkBase.ControlType.kPosition);
            rightClimberPIDController.setReference(Constants.ClimberPositions.ClimbDeepCageRight, SparkBase.ControlType.kPosition);
        }

    /* Create Manual and Set Position commands below - 
    these are called from the main execution file to actually run the methods declared above */
        
        // Manual Commands
    
        public Command climberStopCommand(){
            return run(() -> climberStop());
        }

        public Command climberUpCommand(){
            return run(() -> climberUp());
        }
    
        public Command climberDownCommand(){
            return run(() -> climberDown());
        }

        public Command climbDrivePositionCommand(){
            return run(() -> climbDrivePosition()).until(() -> ((Math.abs(leftClimberEncoder.getPosition() - Constants.ClimberPositions.ClimbDriveLeft) < 1) && (Math.abs(rightClimberEncoder.getPosition() - Constants.ClimberPositions.ClimbDriveRight) < 1)));
        }

        //Set Position Commands
    
        public Command ClimbDeepCageCommand(){
            return run(() -> ClimbDeepCage()).until(() -> ((Math.abs(leftClimberEncoder.getPosition() - Constants.ClimberPositions.ClimbDeepCageLeft) < 1) && (Math.abs(rightClimberEncoder.getPosition() - Constants.ClimberPositions.ClimbDeepCageRight) < 1)));
        }

    @Override
    public void periodic(){
      SmartDashboard.putNumber("rightClimberPosition",rightClimberEncoder.getPosition());
      SmartDashboard.putNumber("leftClimberPosition",leftClimberEncoder.getPosition());
    }
    }
    
