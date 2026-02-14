package frc.robot.vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization extends SubsystemBase{
    public static final Pose2d redHub = new Pose2d();
    public static final Pose2d blueHub = new Pose2d();

    public static final Pose2d[] redLobTargets = {new Pose2d(), new Pose2d()};
    public static final Pose2d[] blueLobTargets = {new Pose2d(), new Pose2d()};

    private static ArrayList<RedRockCamera> cameras = new ArrayList<>();

    public static ArrayList<RRPoseEstimate> getPoseEstimates() {
        ArrayList<RRPoseEstimate> estimates = new ArrayList<>();

        for (RedRockCamera camera : cameras) {
            if (!camera.hasValidPoseEstimate()) continue;
            RRPoseEstimate est = new RRPoseEstimate();
            EstimatedRobotPose pvEst = camera.getEstimate();
            est.pose = pvEst.estimatedPose.toPose2d();
            est.timeStamp = pvEst.timestampSeconds;
            est.stdvs = camera.getStdvs();
            estimates.add(est);
        }
        
        return estimates;
    }


    public Localization() {
        super("localization");

        // SmartDashboard.putNumber("test", 0);

        cameras.add(new RedRockCamera("Photon-Rubik-Everything")
            .withRobotToCameraTransform(
                new Transform3d(
                    new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(2.45), Units.inchesToMeters(6.75)),//new Translation3d(-0.26035, 0.06223, 0.171415),
                    new Rotation3d(Math.toRadians(0), Math.toRadians(33.2), Math.toRadians(180 - 36.2))//new Rotation3d(0, 0.5794493, 2.5097835)//
                )
            )
        );
        cameras.add(new RedRockCamera("Photon-Rubik-Nothing")
            // .withRobotToCameraTransform(
            //     new Transform3d(
            //         new Translation3d(-0.26035, -0.06223, 0.171415),
            //         new Rotation3d(0,0.5794493,3.7734018)
            //     )
            // )
        );
    }

    /**
     * used to update cameras in an instance and nothing else
     */
    @Override
    public void periodic() {
        for (RedRockCamera camera : cameras) camera.update();
    }

    public static class RRPoseEstimate {
        public Pose2d pose;
        public double timeStamp;
        public Matrix<N3, N1> stdvs;
    }
}