package redrocklib.wrappers;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RedRockCamera {
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    public final AprilTagFieldLayout fieldLayout;
    public Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());

    private PhotonPipelineResult result;

    public String name;
    
    public RedRockCamera(String cameraName, AprilTagFieldLayout layout) {
        name = cameraName;
        camera = new PhotonCamera(name);

        fieldLayout = layout;
        poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    }

    public RedRockCamera withRobotToCameraTransform(Transform3d transform) {
        robotToCamera = transform;
        poseEstimator.setRobotToCameraTransform(robotToCamera);
        return this;
    }

    public void update() {
        result = camera.getLatestResult();
        
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }
        }

        SmartDashboard.putBoolean(name + "/" + name + "-has-targets", result.hasTargets());
    }
}
