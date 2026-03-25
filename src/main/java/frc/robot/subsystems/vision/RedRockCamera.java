package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import redrocklib.logging.SmartDashboardNumber;

public class RedRockCamera {
    public static final boolean kEnableCameraTuning = true;
    public static final SmartDashboardNumber kMaxDistToTag = new SmartDashboardNumber("localization/max dist", 5, true && kEnableCameraTuning);
    public static final SmartDashboardNumber kMaxAmbiguityThreshold = new SmartDashboardNumber("localization/max ambiguity", 0.15, true && kEnableCameraTuning);
    
    public static final RRStdv kDefaultStdvs = new RRStdv(0.8, 0.8, 2);
    public static final AprilTagFieldLayout defaultFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public final PhotonCamera camera;
    public final PhotonPoseEstimator poseEstimator;
    public final AprilTagFieldLayout fieldLayout;
    private Transform3d robotToCamera = new Transform3d(new Translation3d(), new Rotation3d());
    private SmartDashboardNumber stdvX, stdvY, stdvTheta;

    @SuppressWarnings("unused")
    private Translation3d robotToCameraTranslation = new Translation3d();
    @SuppressWarnings("unused")
    private Rotation3d robotToCameraRotation = new Rotation3d();

    private Optional<EstimatedRobotPose> visionEst = Optional.empty();

    public String name;
    private boolean targetFound = false;
    private double distToTag = 0;
    private Field2d field2d = new Field2d();
    private double lowestAmbiguity = 10;

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
        robotToCameraTranslation = transform.getTranslation();
        robotToCameraRotation = transform.getRotation();
        poseEstimator.setRobotToCameraTransform(robotToCamera);
        return this;
    }

    public Optional<EstimatedRobotPose> getEstimate() {
        return visionEst;
    }

    public boolean hasValidPoseEstimate() {
        return visionEst.isPresent() && hasTarget() && distToTag < kMaxDistToTag.getNumber() && lowestAmbiguity < kMaxAmbiguityThreshold.getNumber();
    }

    public boolean hasTarget() {
        return visionEst.isPresent();
    }

    public Matrix<N3, N1> getStdvs() {
        return VecBuilder.fill(
            adjustStdv(stdvX.getNumber(), distToTag), 
            adjustStdv(stdvY.getNumber(), distToTag), 
            adjustStdv(stdvTheta.getNumber(), distToTag));
    }

    public void update() {        
        visionEst = Optional.empty();

        targetFound = false;

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (!result.hasTargets() || result.getTimestampSeconds() < 0) continue;
            visionEst = poseEstimator.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()) {
                visionEst = poseEstimator.estimateLowestAmbiguityPose(result);

                double lowestAmbiguityScore = 10;
                PhotonTrackedTarget lowestAmbiguityTarget = null;

                if (!visionEst.isEmpty()) {
                    for (PhotonTrackedTarget target : visionEst.get().targetsUsed) {
                        double targetPoseAmbiguity = target.getPoseAmbiguity();
                        // Make sure the target is a Fiducial target.
                        if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                            lowestAmbiguityScore = targetPoseAmbiguity;
                            lowestAmbiguityTarget = target;
                        }
                    }
                }
                distToTag = (lowestAmbiguityTarget == null) ? distToTag : lowestAmbiguityTarget.getBestCameraToTarget().getTranslation().getNorm();
                this.lowestAmbiguity = lowestAmbiguityScore;

            } else {
                double sum = 0;
                var targetsUsed = visionEst.get().targetsUsed;

                double lowestMultiTagAmbiguity = 10;

                for (PhotonTrackedTarget target : targetsUsed) {
                    double targetPoseAmbiguity = target.getPoseAmbiguity();
                    if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestMultiTagAmbiguity) lowestMultiTagAmbiguity = targetPoseAmbiguity;
                    if (target.fiducialId > 0) {
                        sum += (1.0 / target.getBestCameraToTarget().getTranslation().getNorm());
                    }
                }

                distToTag = targetsUsed.size() * (1.0 / sum);
                this.lowestAmbiguity = lowestMultiTagAmbiguity;
            }


            // boolean useMultitag = multiResult.isPresent();

            // if (useMultitag) {
            //     var best_tf = result.getMultiTagResult().get().estimatedPose.best;
            //     var best =
            //                     Pose3d.kZero
            //                             .plus(best_tf) // field-to-camera
            //                             .relativeTo(fieldLayout.getOrigin())
            //                             .plus(robotToCamera.inverse()); // field-to-robot
            //     distToTag = best_tf.getTranslation().getNorm();
            //     visionEst = Optional.of(
            //                     new EstimatedRobotPose(
            //                             best,
            //                             result.getTimestampSeconds(),
            //                             result.getTargets(),
            //                             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
            // } else {
            //     poseEstimator.estimateLowestAmbiguityPose(result);
            //     PhotonTrackedTarget lowestAmbiguityTarget = null;

            //     double lowestAmbiguityScore = 10;

            //     for (PhotonTrackedTarget target : result.targets) {
            //         double targetPoseAmbiguity = target.getPoseAmbiguity();
            //         // Make sure the target is a Fiducial target.
            //         if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
            //             lowestAmbiguityScore = targetPoseAmbiguity;
            //             lowestAmbiguityTarget = target;
            //         }
            //     }

            //     // Although there are confirmed to be targets, none of them may be fiducial
            //     // targets.
            //     if (lowestAmbiguityTarget == null) {
            //         visionEst = Optional.empty();
            //         continue;
            //     }

            //     int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

            //     Optional<Pose3d> targetPosition = fieldLayout.getTagPose(targetFiducialId);

            //     if (targetPosition.isEmpty()) {
            //         // reportFiducialPoseError(targetFiducialId);
            //         visionEst = Optional.empty();
            //         continue;
            //     }

            //     distToTag = lowestAmbiguityTarget.getBestCameraToTarget().getTranslation().getNorm();

            //     visionEst = Optional.of(
            //             new EstimatedRobotPose(
            //                     targetPosition
            //                             .get()
            //                             .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
            //                             .transformBy(robotToCamera.inverse()),
            //                     result.getTimestampSeconds(),
            //                     result.getTargets(),
            //                     PoseStrategy.LOWEST_AMBIGUITY));
            // }
        }

        this.targetFound = !visionEst.isEmpty();

        SmartDashboard.putBoolean(name + "/" + name + "-has-targets", this.targetFound);
        SmartDashboard.putNumber(name + "/" + name + "-distance-to-tag", distToTag);
        SmartDashboard.putNumber(name + "/" + name + "-lowest-ambiguity", this.lowestAmbiguity);

        if (targetFound) field2d.setRobotPose(getEstimate().get().estimatedPose.toPose2d());
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
