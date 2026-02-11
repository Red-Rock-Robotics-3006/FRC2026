package redrocklib.wrappers;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RedRockCamera {
    public static final AprilTagFieldLayout defaultFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    public final AprilTagFieldLayout fieldLayout;
    public Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());

    private PhotonPipelineResult result;

    private Optional<EstimatedRobotPose> visionEst = Optional.empty();

    public String name;

    private boolean targetFound = false;

    private double distToTag = 0;

    public RedRockCamera(String cameraName) {
        this(cameraName, defaultFieldLayout);
    }
    
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

    public EstimatedRobotPose getEstimate() {
        return visionEst.get();
    }

    public boolean hasTarget() {
        return this.targetFound;
    }

    public Matrix<N3, N1> getStdvs() {
        return VecBuilder.fill(0, 0, 0);
    }

    public void update() {
        result = camera.getLatestResult();
        
        visionEst = Optional.empty();

        targetFound = false;

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets() || result.getTimestampSeconds() < 0) continue;
            var multiResult = result.getMultiTagResult();
            poseEstimator.estimateCoprocMultiTagPose(result);
            boolean useMultitag = multiResult.isPresent();

            if (useMultitag) {
                var best_tf = result.getMultiTagResult().get().estimatedPose.best;
                        var best =
                                Pose3d.kZero
                                        .plus(best_tf) // field-to-camera
                                        .relativeTo(fieldLayout.getOrigin())
                                        .plus(robotToCamera.inverse()); // field-to-robot
                double distToTag = best_tf.getTranslation().getNorm();
                visionEst = Optional.of(
                                new EstimatedRobotPose(
                                        best,
                                        result.getTimestampSeconds(),
                                        result.getTargets(),
                                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
            }
        }

        this.targetFound = !visionEst.isEmpty();

        SmartDashboard.putBoolean(name + "/" + name + "-has-targets", this.targetFound);
        // SmartDashboard.putBoolean(name + "/" + name + "-has-targets", result.hasTargets());
    }
}
