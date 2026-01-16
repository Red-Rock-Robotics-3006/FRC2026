package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();

    public Command testAutoPaths() {
        return drivetrain.followTrajectory("TestPath2026");
    }

    public Command slowTestAutoPaths() {
        return drivetrain.followTrajectory("TestPathSlow");
    }
}
