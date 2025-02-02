// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;

public class Swerve extends SubsystemBase {
  private final AHRS m_gyro;

  private SwerveModule[] mSwerveMods;
  private SwerveDrivePoseEstimator swerveOdometry;
  public Field2d field;
  public RobotConfig config;

  public Vision photonVision = new Vision();


  public Swerve(Vision v) {
    this.photonVision = v;
    m_gyro = new AHRS(NavXComType.kMXP_SPI); // FROM PUBLIC CLASS AHRS DAN_F *Changed to updated studica syntax By B.Mcc
    // .configFactoryDefault();
    zeroHeading();
    mSwerveMods = new SwerveModule[] {
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

    swerveOdometry = new SwerveDrivePoseEstimator(// FROM CLASS SwerveDrivePoseEstimator DAN_F
        Constants.Swerve.swerveKinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionStdDevs);

    SmartDashboard.putData("Field", field);
    // AutoBuilder.configureHolonomic(//FROM CLASS AUTOBUILDER DAN_F
    // this::getPose, // Robot pose supplier
    // this::resetPose, // Method to reset odometry (will be called if your auto has
    // a starting pose)
    // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
    // RELATIVE
    // this::driveRobotRelative, // Method that will drive the robot given ROBOT
    // RELATIVE ChassisSpeeds
    // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
    // likely live in your Constants class
    // new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI,
    // Constants.Swerve.driveKD), // Translation PID constants
    // new PIDConstants(5, Constants.Swerve.angleKI, Constants.Swerve.angleKD), //
    // Rotation PID constants
    // 4, // Max module speed, in m/s
    // 0.4, // Drive base radius in meters. Distance from robot center to furthest
    // module.
    // new ReplanningConfig() //NO DECLARATION DAN_F Default path replanning config.
    // See the API for the options here
    // ),
    // () -> {
    // // Boolean supplier that controls when the path will be mirrored for the red
    // alliance
    // // This will flip the path being followed to the red side of the field.
    // // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   
    // Load the RobotConfig from the GUI settings. You should probably store this in your Constants file
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(Constants.Swerve.driveKP, Constants.Swerve.driveKI,Constants.Swerve.driveKD), // Translation PID constants
              new PIDConstants(Constants.Swerve.angleKP, Constants.Swerve.angleKI,Constants.Swerve.angleKD) // Rotation PID constants
      ),
      config, // The robot configuration
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

  public void drive(Translation2d translation, double rotation, /* boolean fieldRelative, */ boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(), translation.getY(), rotation, getRotation2d()));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /*
   * Drive with field relative boolean
   * public void drive(
   * Translation2d translation, double rotation, boolean fieldRelative, boolean
   * isOpenLoop) {
   * SwerveModuleState[] swerveModuleStates =
   * Constants.Swerve.swerveKinematics.toSwerveModuleStates(
   * fieldRelative
   * ? ChassisSpeeds.fromFieldRelativeSpeeds(
   * translation.getX(), translation.getY(), rotation, getYaw())
   * : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
   * SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
   * Constants.Swerve.maxSpeed);
   * 
   * for (SwerveModule mod : mSwerveMods) {
   * mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
   * }
   * }
   */

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();// NO DECLARATION FOR SWERVEODOMETRY
  }

  public void resetPose(Pose2d pose) {
  swerveOdometry.resetPosition(getRotation2d(), getModulePositions(),
  pose);//NO DECLARATION FOR SWERVEODOMETRY
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
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

  // public void resetOdometry(Pose2d pose) {
  // swerveOdometry.resetPosition(getRotation2d(), getModulePositions(),
  // pose);//NO DECLARATION FOR SWERVEODOMETRY
  // }

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
  /*
   * I think this was added by AJ because he couldn't solve a problem with
   * pathplanner
   * Probably a better solution, commenting out until we know we need it -B.McC
   */
  // public void setHeading(){
  // m_gyro.setAngleAdjustment(180);
  // }

  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360);
  }

  public double getGyroRoll() {
    return m_gyro.getRoll();
  }

  /*
   * TODO: Test this to see if it is giving correct values on SmartDashboard (called in periodic)
   * If incorrect values, try switching to the commented out code below this that calls from getHeading
   */
  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }
  /*
   * public Rotation2d getRotation2d() {
   * return Rotation2d.fromDegrees(getHeading());
   * }
   */

  /*
   * public Rotation2d getYaw() {
   * return (Constants.Swerve.invertGyro)
   * ? Rotation2d.fromDegrees(360 - gyro.getYaw())
   * : Rotation2d.fromDegrees(gyro.getYaw());
   * }
   */

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
      System.out.println("Modules Reset to Absolute");
    }
  }

  /* TODO: code that will only allow position based on AprilTag if we get good results
  public boolean isEstimateValid(Optional<EstimatedRobotPose>
  visionEstimateFront){ 
  if(visionEstimateFront == null){
  return false;
  }
  //reject poses that aren't inside the field
  boolean isInField = visionEstimateFront.get().estimatedPose.getX() > 0 &&
  visionEstimateFront.get().estimatedPose.getX() < FieldConstants.FIELD_LENGTH
  && visionEstimateFront.get().estimatedPose.getY() > 0 &&
  visionEstimateFront.get().estimatedPose.getY() < FieldConstants.FIELD_WIDTH;
  //reject tags > 5m away
  boolean isCloseEnough = visionEstimateFront.get().targetsUsed. < 5;
  //reject pose estimates with no tags in view
  boolean canSeeTag = visionEstimateFront.get().targetsUsed. .tagCount > 0;
  return canSeeTag && isInField && isCloseEnough;
  }
  

  private boolean isOdometryValid(){ 
  giving data
  for(SwerveModule module: swerveModules){
  if(!module.isEncoderDataValid()){
  return false;
  }
  }
  return true;
  }
  */

  // check if the robot thinks it's outside the field, and move it back if it is
  private void keepOdometryOnField() {
    Pose2d pose = getPose();
    double x = pose.getX();
    double y = pose.getY();
    Rotation2d heading = pose.getRotation();
    boolean isInField = true;

    if (x < 0) {
      x = 0;
      isInField = false;
    }
    if (y < 0) {
      y = 0;
      isInField = false;
    }
    if (x > FieldConstants.FIELD_LENGTH) {
      x = FieldConstants.FIELD_LENGTH;
      isInField = false;
    }
    if (y > FieldConstants.FIELD_WIDTH) {
      y = FieldConstants.FIELD_WIDTH;
      isInField = false;
    }
    if (!isInField) {
      swerveOdometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d(x, y, heading));
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2d(), getModulePositions());

    // Correct pose estimate with vision measurements
    SmartDashboard.putNumber("poseY", getPose().getY());
    SmartDashboard.putNumber("poseX", getPose().getX());
    SmartDashboard.putNumber("NAVX Heading", this.getRotation2d().getDegrees());

    Optional<EstimatedRobotPose> visionEstimate = photonVision.getEstimatedPose();// FROM VISION.JAVA

    if (visionEstimate.isPresent()) {
      swerveOdometry.addVisionMeasurement(visionEstimate.get().estimatedPose.toPose2d(),
          visionEstimate.get().timestampSeconds);

      SmartDashboard.putNumber("Front Estimate X", visionEstimate.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Front Estimate Y", visionEstimate.get().estimatedPose.toPose2d().getY());

    }
    SmartDashboard.putBoolean("Front Estimate Present", visionEstimate.isPresent());
    SmartDashboard.putBoolean("Front AprilTag Present", visionEstimate.isPresent());

    keepOdometryOnField();
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANCoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANCoder * 360)", mod.getCanCoder360().getDegrees());
    }

  }
}