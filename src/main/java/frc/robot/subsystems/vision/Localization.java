package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.autoaim.SOTMCalcs;

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

    public static final Transform3d kTurretToLimelightTransform = 
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(7.053), 
                                    Units.inchesToMeters(0), 
                                    Units.inchesToMeters(19.212751)),
                    new Rotation3d(0, Math.toRadians(-28), 0)
                );

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

    public static void setCameraDynamicRotation(Translation2d shooterOffset, Rotation2d turretRotation, Rotation2d drivetrainRotation, int cameraIndex) {
        double[] turretToRobot = SOTMCalcs.rotate(shooterOffset.getX(), shooterOffset.getY(), drivetrainRotation);
        double[] llToTurret = SOTMCalcs.rotate(kTurretToLimelightTransform.getX(), kTurretToLimelightTransform.getY(), drivetrainRotation.plus(turretRotation));

        cameras.get(cameraIndex)
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(
                        turretToRobot[0] + llToTurret[0],
                        turretToRobot[1] + llToTurret[1],
                        kTurretToLimelightTransform.getZ()
                    ),
                    kTurretToLimelightTransform.getRotation()
                        .rotateBy(new Rotation3d(
                            drivetrainRotation.plus(turretRotation)
                        ))
                )
            );
    }

    public static void setCameraDynamicRotation(Translation2d shooterOffset, Rotation2d turretRotation, Rotation2d drivetrainRotation) {
        setCameraDynamicRotation(shooterOffset, turretRotation, drivetrainRotation, kTurretLLIndex);
    }

    private Localization() {
        super("localization");

        //thriftycam on right side (loom retractor)
        cameras.add(new RedRockCamera("Photon-Rubik-Everything")
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.376042), 
                                    Units.inchesToMeters(2.325621), 
                                    Units.inchesToMeters(7.979991)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0))
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(-155)))
                    )
            )
        );

        //thriftycam on left side (breaker)
        cameras.add(new RedRockCamera("Photon-Rubik-Nothing") 
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.417663), 
                                    Units.inchesToMeters(-4.293691), 
                                    Units.inchesToMeters(12.979991)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0))
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(150)))
                )
            )
        );

        //limelight 3g on turret
        cameras.add(new RedRockCamera("Photon-Rubik-Booger") 
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(7.053), 
                                    Units.inchesToMeters(0), 
                                    Units.inchesToMeters(19.212751)),
                    new Rotation3d(0, Math.toRadians(-28), 0)
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
