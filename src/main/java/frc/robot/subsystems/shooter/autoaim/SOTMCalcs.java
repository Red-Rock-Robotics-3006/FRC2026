package frc.robot.subsystems.shooter.autoaim;

import edu.wpi.first.math.geometry.Translation2d;

public class SOTMCalcs {

    private static final double kFudgeFactor = 1;
    
    public static Translation2d getOffset(double velocityX, double velocityY) {
        return new Translation2d(-kFudgeFactor * velocityX, -kFudgeFactor * velocityY);
    }
}
