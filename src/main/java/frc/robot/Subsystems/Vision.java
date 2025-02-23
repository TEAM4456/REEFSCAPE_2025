package frc.robot.Subsystems;

import java.lang.reflect.Array;
import java.util.ArrayList;
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

    //ARRAY OF PHOTONCAMERAS
    public class PhotonGroup {
        PhotonCamera camera; 
        PhotonPoseEstimator estimator;
        public PhotonGroup(PhotonCamera camera, PhotonPoseEstimator estimator){
            this.camera=camera;
            this.estimator=estimator;
            // TODO maybe we could do here instead of explicit loop in Vision()
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
    };

    
    //public final PhotonPoseEstimator photonPoseEstimator;
    PhotonGroup[] cameras = 
    {
        new PhotonGroup (
            new PhotonCamera("limelight front"), 
            new PhotonPoseEstimator(fieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                Constants.VisionConstants.ROBOT_TO_LIMELIGHT2)
        ),
        new PhotonGroup (
            new PhotonCamera("limelight angle"), 
            new PhotonPoseEstimator(fieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                Constants.VisionConstants.ROBOT_TO_LIMELIGHT1)
        )
    };
    public Vision(){
       
        // we hopefully cna do it in the constructor
        //for (PhotonGroup cam: cameras){
        //    cam.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //}
        SmartDashboard.putString("Vision","Vision initialization");
    }

    public ArrayList<Optional<EstimatedRobotPose>> getEstimatedPose() {
        SmartDashboard.putString("Vision","getEstimatePose");
        SmartDashboard.putString("NULLS","checking for NULL");

        ArrayList<Optional<EstimatedRobotPose>> robotPoses = new ArrayList<>();
        if (cameras == null){       
             SmartDashboard.putString("NULLS","cameras are null");
        }   
        for (PhotonGroup cameraGroup: cameras)
        {
            if (cameraGroup == null){       
                SmartDashboard.putString("NULLS","cameraGroup is null");
            }   
            PhotonPipelineResult result = cameraGroup.camera.getLatestResult();
            if(result.hasTargets()){
                Optional<EstimatedRobotPose> robotPose = cameraGroup.estimator.update(result);
        
                /* TODO: Test the commented out code below
                // Get the ambiguity value and display it on the SmartDashboard
                double ambiguity = result.getBestTarget().getPoseAmbiguity();
                SmartDashboard.putNumber("Pose Ambiguity", ambiguity); */

                robotPoses.add(robotPose);
            }

            /* TODO: Test the commented out code below
            // Display a default value when no targets are found
            SmartDashboard.putNumber("Pose Ambiguity", -1.0); */
            /* 
            Optional<EstimatedRobotPose> emptyPose = Optional.empty();
             return emptyPose;
            */
        }
        return robotPoses;
    }

    public void periodic() {
        SmartDashboard.putString("Vision","about to get EstimatePose");
        getEstimatedPose();
    }
    

}