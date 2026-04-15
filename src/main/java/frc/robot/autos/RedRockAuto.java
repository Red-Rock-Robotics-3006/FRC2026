package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RedRockAuto {
    private String autoName;
    private Command autoCommand;
    private Pose2d autoBlueInitialPose;
    private Pose2d autoRedInitialPose;

    public RedRockAuto(String name, Command command, Pose2d blueInitialPose, Pose2d redInitialPose) {
        this.autoName = name;
        this.autoCommand = command;
        this.autoBlueInitialPose = blueInitialPose;
        this.autoRedInitialPose = redInitialPose;
    }

    public String getName() {
        return this.autoName;
    }

    public Command getCommand() {
        return this.autoCommand;
    }

    public Pose2d getBlueInitialPose() {
        return this.autoBlueInitialPose;
    }

    public Pose2d getRedInitialPose() {
        return this.autoRedInitialPose;
    }
}