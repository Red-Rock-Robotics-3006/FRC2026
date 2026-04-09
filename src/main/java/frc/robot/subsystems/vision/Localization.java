package frc.robot.subsystems.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
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

    public static final int kTurretLLIndex = 3;

    public static final Pose2d blueHub = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));
    public static final Pose2d redHub = new Pose2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84), Rotation2d.fromDegrees(0));

    // first pose is UPPER lob pose, second pose is LOWER lob pose
    public static final Pose2d[] redLobTargets = {
        new Pose2d(15.56, 6.84, Rotation2d.fromDegrees(0)),
        new Pose2d(15.56, 1.25, Rotation2d.fromDegrees(0))};
    
    public static final Pose2d[] blueLobTargets = {
        new Pose2d(1, 6.84, Rotation2d.fromDegrees(0)),
        new Pose2d(1, 1.25, Rotation2d.fromDegrees(0))};

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
        if (cameraIndex >= cameras.size()) return;
        double[] turretToLLxy = SOTMCalcs.rotate(kTurretToLimelightTransform.getX(), kTurretToLimelightTransform.getY(), turretRotation);

        cameras.get(cameraIndex)
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(
                        shooterOffset.getX() + turretToLLxy[0],
                        shooterOffset.getY() + turretToLLxy[1],
                        kTurretToLimelightTransform.getZ()
                    ),
                    kTurretToLimelightTransform.getRotation()
                        .rotateBy(new Rotation3d(turretRotation))
                )
            );
    }

    public static void setCameraDynamicRotation(Translation2d shooterOffset, Rotation2d turretRotation, Rotation2d drivetrainRotation) {
        setCameraDynamicRotation(shooterOffset, turretRotation, drivetrainRotation, kTurretLLIndex);
    }

    private Localization() {
        super("localization");

        //thriftycam on right side
        cameras.add(new RedRockCamera("Everything")
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-9.150000), Units.inchesToMeters(-14.052404), Units.inchesToMeters(8.503539)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0)) //pitch
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(-90))) //yaw
                    )
            )
        );

        //thriftycam on back
        cameras.add(new RedRockCamera("Something")
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-11.612985), Units.inchesToMeters(-5.750000), Units.inchesToMeters(7.167866)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0)) //pitch
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(180))) //yaw    
                    )
            )
        );

        //thriftycam on left side
        cameras.add(new RedRockCamera("Nothing") 
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-9.150000), Units.inchesToMeters(14.052404), Units.inchesToMeters(8.503539)),
                    new Rotation3d()
                        .rotateBy(new Rotation3d(0, Math.toRadians(-33.2), 0)) //pitch
                        .rotateBy(new Rotation3d(Rotation2d.fromDegrees(90))) //yaw
                )
            )
        );

        //thriftycam on turret
        cameras.add(new RedRockCamera("Good")
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(6.576997), Units.inchesToMeters(0), Units.inchesToMeters(20.513622)),
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
