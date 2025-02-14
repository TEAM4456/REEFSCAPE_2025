package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    //private final PhotonCamera camera = new PhotonCamera("limelight angle");
    //public final PhotonPoseEstimator photonPoseEstimatorAngle;
    private AprilTagFieldLayout fieldLayout;

    private final PhotonCamera camera = new PhotonCamera("limelight front");
    public final PhotonPoseEstimator photonPoseEstimator;
    
    public Vision(){


        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.ROBOT_TO_LIMELIGHT2);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        PhotonPipelineResult result = camera.getLatestResult();
        if(result.hasTargets()){
         Optional<EstimatedRobotPose> robotPose = photonPoseEstimator.update(result);
         return robotPose;
                  
            
        }
        Optional<EstimatedRobotPose> emptyPose = Optional.empty();
        return emptyPose;
    }

    public void periodic() {
        getEstimatedPose();
    }
    

}