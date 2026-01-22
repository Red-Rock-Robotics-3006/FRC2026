package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class Localization {
    public static final Pose2d redHub = new Pose2d();
    public static final Pose2d blueHub = new Pose2d();

    public static final Pose2d[] redLobTargets = {new Pose2d(), new Pose2d()};
    public static final Pose2d[] blueLobTargets = {new Pose2d(), new Pose2d()};
}
