package frc.robot.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redrocklib.logging.SmartDashboardNumber;

public class RedRockCamera {
    public static final boolean kEnableCameraTuning = true;
    public static final SmartDashboardNumber kMaxDistToTag = new SmartDashboardNumber("localization-max dist", 3, true && kEnableCameraTuning);
    
    public static final RRStdv kDefaultStdvs = new RRStdv(0.8, 0.8, 2);
    public static final AprilTagFieldLayout defaultFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    public final AprilTagFieldLayout fieldLayout;
    public Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());

    private SmartDashboardNumber stdvX, stdvY, stdvTheta;

    private Optional<EstimatedRobotPose> visionEst = Optional.empty();

    public String name;

    private boolean targetFound = false;

    private double distToTag = 0;

    private Field2d field2d = new Field2d();

    public RedRockCamera(String cameraName) {
        this(cameraName, defaultFieldLayout, kDefaultStdvs);
    }

    public RedRockCamera(String cameraName, RRStdv stdvs) {
        this(cameraName, defaultFieldLayout, stdvs);
    }
    
    public RedRockCamera(String cameraName, AprilTagFieldLayout layout, RRStdv stdvs) {
        name = cameraName;
        camera = new PhotonCamera(name);

        fieldLayout = layout;
        poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

        stdvX = new SmartDashboardNumber(name + "/" + name + "-stdv/stdvX", stdvs.stdvX, true && kEnableCameraTuning);
        stdvY = new SmartDashboardNumber(name + "/" + name + "-stdv/stdvY", stdvs.stdvY, true && kEnableCameraTuning);
        stdvTheta = new SmartDashboardNumber(name + "/" + name + "-stdv/stdvTheta", stdvs.stdvTheta, true && kEnableCameraTuning);

        SmartDashboard.putData(name + "/" + name + "-field", field2d);
    }

    public RedRockCamera withRobotToCameraTransform(Transform3d transform) {
        robotToCamera = transform;
        poseEstimator.setRobotToCameraTransform(robotToCamera);
        return this;
    }

    public EstimatedRobotPose getEstimate() {
        return visionEst.get();
    }

    public boolean hasValidPoseEstimate() {
        return hasTarget() && distToTag < kMaxDistToTag.getNumber() && visionEst.isPresent();
    }

    public boolean hasTarget() {
        return this.targetFound;
    }

    public Matrix<N3, N1> getStdvs() {
        return VecBuilder.fill(
            adjustStdv(stdvX.getNumber(), distToTag), 
            adjustStdv(stdvY.getNumber(), distToTag), 
            adjustStdv(stdvTheta.getNumber(), distToTag));
    }

    public void update() {        
        visionEst = Optional.empty();

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
                distToTag = best_tf.getTranslation().getNorm();
                visionEst = Optional.of(
                                new EstimatedRobotPose(
                                        best,
                                        result.getTimestampSeconds(),
                                        result.getTargets(),
                                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
            } else {
                // poseEstimator.estimateLowestAmbiguityPose(result);
                PhotonTrackedTarget lowestAmbiguityTarget = null;

                double lowestAmbiguityScore = 10;

                for (PhotonTrackedTarget target : result.targets) {
                    double targetPoseAmbiguity = target.getPoseAmbiguity();
                    // Make sure the target is a Fiducial target.
                    if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                        lowestAmbiguityScore = targetPoseAmbiguity;
                        lowestAmbiguityTarget = target;
                    }
                }

                // Although there are confirmed to be targets, none of them may be fiducial
                // targets.
                if (lowestAmbiguityTarget == null) {
                    visionEst = Optional.empty();
                    continue;
                }

                int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

                Optional<Pose3d> targetPosition = fieldLayout.getTagPose(targetFiducialId);

                if (targetPosition.isEmpty()) {
                    // reportFiducialPoseError(targetFiducialId);
                    visionEst = Optional.empty();
                    continue;
                }

                distToTag = lowestAmbiguityTarget.getBestCameraToTarget().getTranslation().getNorm();

                visionEst = Optional.of(
                        new EstimatedRobotPose(
                                targetPosition
                                        .get()
                                        .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                                        .transformBy(robotToCamera.inverse()),
                                result.getTimestampSeconds(),
                                result.getTargets(),
                                PoseStrategy.LOWEST_AMBIGUITY));
            }
            break; // only process the first valid result
        }

        this.targetFound = !visionEst.isEmpty();

        SmartDashboard.putBoolean(name + "/" + name + "-has-targets", this.targetFound);
        SmartDashboard.putNumber(name + "/" + name + "-distance-to-tag", distToTag);

        if (targetFound) field2d.setRobotPose(getEstimate().estimatedPose.toPose2d());
        // SmartDashboard.putBoolean(name + "/" + name + "-has-targets", result.hasTargets());
    }

    private double adjustStdv(double stdv, double dist) {
        return stdv + stdv * ((dist * dist) / 20);
    }

    public static class RRStdv {
        public final double stdvX, stdvY, stdvTheta;

        public RRStdv(double sX, double sY, double sTheta) {
            stdvX = sX;
            stdvY = sY;
            stdvTheta = sTheta;
        }
    }
}