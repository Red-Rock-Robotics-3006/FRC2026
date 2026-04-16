package frc.robot.subsystems.shooter.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redrocklib.logging.SmartDashboardNumber;

public class SOTMCalcs {
    public static final boolean kTuning = true;
    public static final SmartDashboardNumber kNewtonIterations = new SmartDashboardNumber("sotm/newton iterations", 100, kTuning && true);

    public static double[] rotate(double x, double y, Rotation2d rotation) {
        return new double[]{
            rotation.getCos() * x - rotation.getSin() * y,
            rotation.getSin() * x + rotation.getCos() * y
        };
    }

    public static Translation2d getSecantOffset(double vx, double vy, Pose2d targetPose, Pose2d currentPose) {
        double t0 = 0;
        Pose2d pose0 = targetPose;

        double t1 = FlightTimeInterpolatingTable.get(distance(pose0, currentPose));
        Pose2d pose1 = pose0.transformBy(
            new Transform2d(
                new Translation2d(
                    -vx * t1,
                    -vy * t1
                ),
                new Rotation2d()
            )
        );

        int iterations = 0;
        double finalTOF = 0;

        for (int i = 0; i < (int)kNewtonIterations.getNumber(); i++) {
            if (Math.abs(t1 - t0) < 1e-5) {
                iterations = i;
                break;
            }
            double ft0 = FlightTimeInterpolatingTable.get(distance(pose0, currentPose));
            double ft1 = FlightTimeInterpolatingTable.get(distance(pose1, currentPose));
            double newTOF = 
                (t0 * (ft1 - t1) - t1 * (ft0 - t0)) / 
                (ft1 - t1 - ft0 + t0);

            t0 = t1;
            t1 = newTOF;

            pose0 = pose1;
            pose1 = targetPose.transformBy(
                new Transform2d(
                    new Translation2d(
                        -vx * newTOF,
                        -vy * newTOF
                    ),
                    new Rotation2d()
                )
            );

            finalTOF = newTOF;
        }
        SmartDashboard.putNumber("sotm/iterations", iterations);
        SmartDashboard.putNumber("sotm/final tof", finalTOF);
        return pose1.minus(targetPose).getTranslation();
    }

    public static double distance(Pose2d pose1, Pose2d pose2) {
        return pose1.minus(pose2).getTranslation().getNorm();
    }
}
