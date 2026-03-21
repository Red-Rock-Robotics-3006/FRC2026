package frc.robot.subsystems.shooter.autoaim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import redrocklib.logging.SmartDashboardNumber;

public class SOTMCalcs {

    public static final double kFlywheelRadiusMeters = 0.0508; // 2-inch radius, tune this

    public static double rpmToExitVelocity(double rpm) { // this assumes 1:1 surface-speed-to-ball-speed transfer, will need to tune
        return (rpm * 2.0 * Math.PI / 60.0) * kFlywheelRadiusMeters;
    }

    public static Translation2d getOffset(double velocityX, double velocityY, double exitVelocity, double distance) {
        if (exitVelocity <= 0) return new Translation2d();

        double flightTime = distance / exitVelocity;
        return new Translation2d(-velocityX * flightTime * kFudgeFactor.getNumber(), -velocityY * flightTime * kFudgeFactor.getNumber());
    }

    public static final SmartDashboardNumber kFudgeFactor = new SmartDashboardNumber("sotm/fudge", 5.67);
    
    public static Translation2d getOffset(double velocityX, double velocityY) {
        return new Translation2d(-kFudgeFactor.getNumber() * velocityX, -kFudgeFactor.getNumber() * velocityY);
    }

    public static double[] rotate(double x, double y, Rotation2d rotation) {
        return new double[]{
            rotation.getCos() * x - rotation.getSin() * y,
            rotation.getSin() * x + rotation.getCos() * y
        };
    }
}
