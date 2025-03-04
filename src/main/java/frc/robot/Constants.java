package frc.robot;

import java.util.HashMap;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.configs.SwerveModuleConstants;


public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.15;

    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(20.5);//wheel to wheel width, not frame to frame
    public static final double wheelBase = Units.inchesToMeters(27);//wheel to wheel length, not frame to frame
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (5.36 / 1.0); // FOR L3+ on the Swerve Drive MK4c
    public static final double angleGearRatio = (12.8 / 1.0); //For MK4c swerve drive

    public static final SwerveDriveKinematics swerveKinematics =
    //XY plane is robot relative with +x is forward (front of robot) and +y is left
      new SwerveDriveKinematics(
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // Mod 0
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // Mod 1
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // Mod 2
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0)); // Mod 3

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int driveContinuousCurrentLimit = 40;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 1.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.467;
    public static final double driveKV = 1.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 3.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false; //not inverted when motors are mounted from above on SDS modules (ex. MK4c)
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /*ASSIGNS THE ANGLE MOTORS, DRIVE MOTORS, AND CAN CODERS OF THE ROBOT */
    /* Back Right Module - Module 0 */
    public static final class Mod0 {
      public static final int angleMotorID = 6;
      public static final int driveMotorID = 5;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(338.64);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 1 */
    public static final class Mod1 {
      public static final int angleMotorID = 8;
      public static final int driveMotorID = 7;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(210.6);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 2 */
    public static final class Mod2 {
      public static final int angleMotorID = 4;
      public static final int driveMotorID = 3;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(140.6);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Left Module - Module 3 */
    public static final class Mod3 {
      public static final int angleMotorID = 2;
      public static final int driveMotorID = 1;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(220.34);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    public int getGyroRoll() {
      return 0;
    }

    public void autoBalance() {
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = .01;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();
  }

  public static final class ElevatorPositions{
    public static final double ElevatorScoreL1 = 0;
    public static final double ElevatorScoreL2 = 0;
    public static final double ElevatorScoreL3 = 0;
    public static final double ElevatorScoreL4 = 0;

    public static final double ElevatorCoralPickupPosition = 0.0;
    public static final double ElevatorClimbPosition = 0.0;
    
    public static final double ElevatorMax = 0;
    public static final double ElevatorMin = 0;

    public static final double elevatorSpeed = .2;

    public static final double ElevatorAlgaeLow = 0.0;
    public static final double ElevatorAlgaeHigh = 0.0;
  }

  public static final class IntakeSpeeds{
    public static final double intakePickupCoral = -150;
    public static final double intakeScoreCoralL2toL4 = 0;
    public static final double intakeScoreCoralL1 = 0;
    public static final double intakeRemoveAlgae = 0;
    public static final double intakePullBack = 0;
    public static final double intakeAutoScore = 0.0;
  }

  //All of these values aren't correct, they are just placeholders
  //Variables have been set up though to be referenced within other classes
  public static final class ElevatorPivotPositions{
  
    public static final double elevatorPivotSpeed = .2;

    public static final double elevatorPivotMax = 0;
    public static final double elevatorPivotMin = 0;

    public static final double elevatorPivotCoralPickupPosition = 0.0;
    public static final double elevatorPivotClimbPosition = 0.0;

    public static final double elevatorPivotScoreL1 = 0.0;
    public static final double elevatorPivotScoreL2 = 0.0;
    public static final double elevatorPivotScoreL3 = 0.0;
    public static final double elevatorPivotScoreL4 = 0.0;

    public static final double elevatorPivotAlgaeLow = 0.0;
    public static final double elevatorPivotAlgaeHigh = 0.0;
    public static final double elevatorPivotDrivePosition = 0.0;
  }
public static final class IntakePivotPositions{
   
    public static final double intakePivotSpeed = .2;

    public static final double intakePivotMax = 0;
    public static final double intakePivotMin = 0;

    public static final double intakePivotCoralPickupPosition = 0.0;
    public static final double intakePivotClimbPosition = 0.0;
    public static final double intakePivotDrivePosition = 0.0;
  
/* REEFSCAPE 2025 CORAL LEVELS (right now it's in inches off ground/arbitrary number so number is lowkey wrong :/ )*/
  public static final double intakePivotScoreL1 = 18;
  public static final double intakePivotScoreL2 = 31.875;
  public static final double intakePivotScoreL3 = 47.625;
  public static final double intakePivotScoreL4 = 72;

  public static final double intakePivotAlgaeHigh = 0.0;
  public static final double intakePivotAlgaeLow = 0.0;
  
}
public static final class AlgaePivotPositions{
  
  public static final double algaePivotSpeed = .2;
  public static final double algaePivotUp = 0.0;
  public static final double algaePivotDown = 0.0;
  public static final double algaePivotScore = 0.0;
  public static final double algaePivotDriveSetting = 0.0;
}
  public static final class ClimberPositions{
    public static final double climberSpeed = .2;
    public static final double climbPosition = .2;
    public static final double ClimbDeepCageLeft = .2;
    public static final double ClimbDeepCageRight = .2;
    public static final double ClimbDriveLeft = .2;
    public static final double ClimbDriveRight = .2;
  }
  
  public static final class AlgaePickupPositions{ 
    public static final double algaePickupSpeed = 0.0;
  }
 

  public static final class VisionConstants {
    public static final Transform3d ROBOT_TO_LIMELIGHT1 = new Transform3d(
      new Translation3d(0, -0.295,0.345),new Rotation3d(Math.toRadians(90), Math.toRadians(180), 0)); //Not sure if Rotation 3D is correct
    public static final Transform3d ROBOT_TO_LIMELIGHT2 = new Transform3d(
        new Translation3d(-0.405, 0, 0.59)/* relative to center of robot (+x value when camera is toward front, -x value when camera is toward back, 
        +y value camera is mounted to left of robot center, -y value when camera is mounted right of robot center, +z value when camera is mounted above ground level) */,
         new Rotation3d(0, Math.toRadians(180), 0)/*y rotation for pitch is 180 degrees when camera faces robot back */);
  }

  public static final class FieldConstants{
    public static final double FIELD_LENGTH = 17.55; //field length in meters
    public static final double FIELD_WIDTH = 8.05; //field width in meters
    //while not technically a constant, the alliance color is used so frequently, we put it in constants so it can be easily accessed from anywhere in the code
    public static boolean isRedAlliance(){
      return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }
    //these functions allow us to mirror a pose2d / translation2d to the correct side of the field to match our alliance color
    public static Translation2d flipTranslationForAlliance(Translation2d translation){
      return isRedAlliance()? new Translation2d( FIELD_LENGTH - translation.getX(), translation.getY()): translation;
    }
    public static Pose2d flipPoseForAlliance(Pose2d pose){
      return isRedAlliance()? new Pose2d( FIELD_LENGTH - pose.getX(), pose.getY(), Rotation2d.fromDegrees(180).minus(pose.getRotation())): pose;
    }
    //pose for scoring in processor

    //poses for scoring coral

  
  }
}