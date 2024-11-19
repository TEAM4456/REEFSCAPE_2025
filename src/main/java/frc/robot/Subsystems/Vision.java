package frc.robot.Subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    //private final PhotonCamera camera = new PhotonCamera("limelight angle");
    //public final PhotonPoseEstimator photonPoseEstimatorAngle;
    private AprilTagFieldLayout fieldLayout;

    private final PhotonCamera camera2 = new PhotonCamera("limelight front");
    public final PhotonPoseEstimator photonPoseEstimatorFront;





    
    public Vision(){


        fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        //photonPoseEstimatorAngle = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.VisionConstants.ROBOT_TO_LIMELIGHT1);
        //photonPoseEstimatorAngle.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonPoseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, Constants.VisionConstants.ROBOT_TO_LIMELIGHT2);
        photonPoseEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
    /* 
    public Optional<EstimatedRobotPose> getEstimatedPoseAngle() {
        PhotonPipelineResult result1 = camera.getLatestResult();
        if(result1.hasTargets()){
            PhotonTrackedTarget target = result1.getBestTarget();
            if ((target.getPoseAmbiguity() <= 0.2 && target.getPoseAmbiguity() != -1 && target.getFiducialId() >=0)){
                if(target.getBestCameraToTarget().getTranslation().getDistance(new Translation3d())<=3){
                    Optional<EstimatedRobotPose> robotPose = photonPoseEstimatorAngle.update();
                    return robotPose;
                }  
            }
        }
        Optional<EstimatedRobotPose> emptyPose = Optional.empty();
        return emptyPose;
    }*/
    public Optional<EstimatedRobotPose> getEstimatedPoseFront() {
        PhotonPipelineResult result2 = camera2.getLatestResult();
        if(result2.hasTargets()){
         Optional<EstimatedRobotPose> robotPose = photonPoseEstimatorFront.update();
         return robotPose;
                  
            
        }
        Optional<EstimatedRobotPose> emptyPose = Optional.empty();
        return emptyPose;
    }

    public void periodic() {
    }
    

}