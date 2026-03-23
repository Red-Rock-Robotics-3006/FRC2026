package frc.robot;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class Autos {

    /*Naming convention:
        [starting position]_[intaking location][shoot or lob]_..._[climb or leave]

        starting position: (L)eft, (C)enter, (R)ight
        intaking location: (M)iddle, (D)epot, (O)utpost
        (S)hoot, (L)ob
        (C)limb, (L)eave
    */

    private static Autos instance = null;

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Climber climber = Climber.getInstance();

    // PATHS AUTOS

    public Command R_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarMidtake"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_MS_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarMidtake"),
            drivetrain.followTrajectory("R_CloseMidtake"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_MS_OS_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarMidtake"),
            drivetrain.followTrajectory("R_Outposttake")
        );
    }

    public Command R_MS_MS_OS_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarMidtake"),
            drivetrain.followTrajectory("R_CloseMidtake"),
            drivetrain.followTrajectory("R_Outposttake")
        );
    }

    public Command M_Depot_Outpost_Paths() {
        return drivetrain.followTrajectory("M_Depot_Outpost");
    }

    public Command TESTPATH() {
        return drivetrain.followTrajectory("Test_path");
    }

    // FULL AUTOS

    public Command R_MS_L() {
        return Commands.sequence(
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_FarMidtake"),
            this.shootAuto(6),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_MS_MS_L() {
        return Commands.sequence(
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_FarMidtake"),
            this.shootAuto(6),

            drivetrain.followTrajectory("R_CloseMidtake"),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_MS_OS() {
        return Commands.sequence(
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_FarMidtake"),
            this.shootAuto(6),

            drivetrain.followTrajectory("R_Outposttake"),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand()
        );
    }

    public Command R_MS_MS_OS() {
        return Commands.sequence(
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_FarMidtake"),
            this.shootAuto(6),

            drivetrain.followTrajectory("R_CloseMidtake"),
            this.shootAuto(4),

            drivetrain.followTrajectory("R_Outposttake"),
            this.shootAuto(7),

            superstructure.intake.stopIntakeCommand()
        );
    }

    // climb sequence if we ever need it ig
    //     
    //         Commands.parallel(
    //             Commands.sequence(
    //                 drivetrain.followTrajectory("R_MS_C", 3),
    //                 drivetrain.pidToPoseAutoCommand(new Pose2d(3006, 3006, Rotation2d.fromDegrees(3006))) //TODO: replace with actual climbing pose this'd be very bad if ran
    //                 ),
    //             climber.raiseClimberCommand().withTimeout(2)
    //         ),
    //         climber.lowerClimberCommand().withTimeout(2),
    //         climber.stopClimberCommand()
    //     );
    // }

    private Command shootAuto(double seconds) {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.FULL_TRACKING),
            Commands.waitSeconds(seconds),
            superstructure.setStateCommand(RobotState.TURRET_TRACKING)
        );
    }

    public static Autos getInstance() {
        if (instance == null) instance = new Autos();
        return instance;
    }
}
