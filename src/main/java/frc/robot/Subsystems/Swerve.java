// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;//NO DECLARATION FOR PHOTONVISION
import org.photonvision.PhotonCamera;//NO DECLARATION FOR PHOTONVISION
import org.photonvision.PhotonPoseEstimator;//NO DECLARATION FOR PHOTONVISION
import org.photonvision.PhotonPoseEstimator.PoseStrategy;//NO DECLARATION FOR PHOTONVISION
import org.photonvision.targeting.PhotonPipelineResult;//NO DECLARATION FOR PHOTONVISION
import org.photonvision.targeting.PhotonTrackedTarget;//NO DECLARATION FOR PHOTONVISION

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;//NO DECLARATION
import com.pathplanner.lib.util.PIDConstants;//NO DECLARATION
import com.pathplanner.lib.util.ReplanningConfig;//NO DECLARATION

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final AHRS m_gyro;

 
  private SwerveModule[] mSwerveMods;
  private SwerveDrivePoseEstimator swervePoseEstimator;
  public Field2d field;

  public Vision photonVision = new Vision();
 
  

  public Swerve(Vision v) {
    this.photonVision = v;
    m_gyro = new AHRS(SPI.Port.kMXP); //FROM PUBLIC CLASS AHRS DAN_F
    //.configFactoryDefault();
    zeroHeading();
    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    Timer.delay(1.0);
    resetModulesToAbsolute();



    field = new Field2d();

        
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5));
    var visionStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(10));
  
     
    swervePoseEstimator =
            new SwerveDrivePoseEstimator(//FROM CLASS SwerveDrivePoseEstimator DAN_F
                    Constants.Swerve.swerveKinematics,
                    getRotation2d(),
                    getModulePositions(),
                    new Pose2d(),
                    stateStdDevs,
                    visionStdDevs);



    SmartDashboard.putData("Field", field);
    AutoBuilder.configureHolonomic(//FROM CLASS AUTOBUILDER DAN_F
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD), // Translation PID constants
            new PIDConstants(5, Constants.Swerve.angleKI, Constants.Swerve.angleKD), // Rotation PID constants
            4, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() //NO DECLARATION DAN_F Default path replanning config. See the API for the options here
        ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }
  public void drive(Translation2d translation, double rotation, /*boolean fieldRelative,*/ boolean isOpenLoop) {
      SwerveModuleState[] swerveModuleStates = 
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2d()));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

/*  Drive with field relative boolean
  public void drive(
    Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
  SwerveModuleState[] swerveModuleStates =
      Constants.Swerve.swerveKinematics.toSwerveModuleStates(
          fieldRelative
              ? ChassisSpeeds.fromFieldRelativeSpeeds(
                  translation.getX(), translation.getY(), rotation, getYaw())
              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

  for (SwerveModule mod : mSwerveMods) {
    mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
  }
}
*/

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();//NO DECLARATION FOR SWERVEODOMETRY 
  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);//NO DECLARATION FOR SWERVEODOMETRY 
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }
  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(targetStates[mod.moduleNumber], true);
    }
  }
  
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);//NO DECLARATION FOR SWERVEODOMETRY 
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroHeading() {
    m_gyro.reset();
  }
  public void zeroHeadingAdjust() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }
  public void setHeading(){
    m_gyro.setAngleAdjustment(180);
  }

  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
}
  public double getGyroRoll(){
    return m_gyro.getRoll();
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }
/*
public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
}
*/

 /* 
  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }
  */

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public void resetModulesToAbsolute() {
      for(SwerveModule mod : mSwerveMods) {
        mod.resetToAbsolute();
        System.out.println("Modules Reset to Absolute");
      }
  }
  
  

 
  @Override
  public void periodic() {
    swervePoseEstimator.update(getRotation2d(), getModulePositions());

     // Correct pose estimate with vision measurements
    SmartDashboard.putNumber("poseY", getPose().getY());
    SmartDashboard.putNumber("poseX", getPose().getX());
    SmartDashboard.putNumber("NAVX Heading", this.getHeading());
    /* 
    Optional<EstimatedRobotPose> visionEstimateAngle = photonVision.getEstimatedPoseAngle();
    if(visionEstimateAngle.isPresent()){
      swervePoseEstimator.addVisionMeasurement(visionEstimateAngle.get().estimatedPose.toPose2d(), visionEstimateAngle.get().timestampSeconds);
      SmartDashboard.putNumber("Angle Estimate X",visionEstimateAngle.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Angle Estimate Y",visionEstimateAngle.get().estimatedPose.toPose2d().getY());

    }*/
    Optional<EstimatedRobotPose> visionEstimateFront = photonVision.getEstimatedPoseFront();//FROM VISION.JAVA
  
    if(visionEstimateFront.isPresent()){
      swervePoseEstimator.addVisionMeasurement(visionEstimateFront.get().estimatedPose.toPose2d(), visionEstimateFront.get().timestampSeconds);

      SmartDashboard.putNumber("Front Estimate X",visionEstimateFront.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Front Estimate Y",visionEstimateFront.get().estimatedPose.toPose2d().getY());
      
      
    }
    SmartDashboard.putBoolean("Front Estimate Present",visionEstimateFront.isPresent());
    SmartDashboard.putBoolean("Front AprilTag",visionEstimateFront.isPresent());


    field.setRobotPose(getPose());
  
   
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
 
  }
}