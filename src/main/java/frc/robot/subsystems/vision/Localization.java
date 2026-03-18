package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase{
    private static Localization instance = null;

    public static final int kTurretLLIndex = 2;

    public static final Pose2d blueHub = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));
    public static final Pose2d redHub = new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));

    // first pose is UPPER lob pose, second pose is LOWER lob pose
    public static final Pose2d[] redLobTargets = {
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
        new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0))};
        
    public static final Pose2d[] blueLobTargets = {
        // new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0)),
        // new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0))};
        new Pose2d(3, 6.4, Rotation2d.fromDegrees(0)),
        new Pose2d(3, 1.7, Rotation2d.fromDegrees(0))};

    private static ArrayList<RedRockCamera> cameras = new ArrayList<>();

    public static ArrayList<RRPoseEstimate> getPoseEstimates() {
        ArrayList<RRPoseEstimate> estimates = new ArrayList<>();

        for (RedRockCamera camera : cameras) {
            if (!camera.hasValidPoseEstimate()) continue;
            RRPoseEstimate est = new RRPoseEstimate();
            EstimatedRobotPose pvEst = null;
            if (camera.getEstimate().isPresent())
                pvEst = camera.getEstimate().get();
            else continue;
            est.pose = pvEst.estimatedPose.toPose2d();
            est.timeStamp = pvEst.timestampSeconds;
            est.stdvs = camera.getStdvs();
            estimates.add(est);
        }
        
        return estimates;
    }

    public static void setCameraDynamicRotation(Rotation2d rotation, int cameraIndex) {
        cameras.get(cameraIndex).withDynamicRotationTransform(rotation);
    }

    public static void setCameraDynamicRotation(Rotation2d rotation) {
        setCameraDynamicRotation(rotation, kTurretLLIndex);
    }

    private Localization() {
        super("localization");

        //right (loom retractor side)
        cameras.add(new RedRockCamera("Photon-Rubik-Everything") //thriftycam on right side
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.376042), Units.inchesToMeters(2.325621), Units.inchesToMeters(7.979991)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0))
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(-155)))
                    )
            )
        );

        //left
        cameras.add(new RedRockCamera("Photon-Rubik-Nothing") //thriftycam on left side
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0))
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(150)))
                )
            )
        );

        //turret
        cameras.add(new RedRockCamera("Photon-Rubik-Booger") //limelight 4 on turret
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
                    new Rotation3d(new Quaternion(0, 0, 0, 0))
                )
            )
        );
    }

    /**
     * used to update cameras in an instance and nothing else
     */
    @Override
    public void periodic() {
        for (RedRockCamera camera : cameras) camera.update();
    }

    public static Localization getInstance() {
        if (instance == null) instance = new Localization();
        return instance;
    }

    public static class RRPoseEstimate {
        public Pose2d pose;
        public double timeStamp;
        public Matrix<N3, N1> stdvs;
    }
}
