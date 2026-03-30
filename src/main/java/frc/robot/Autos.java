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

        starting position: (L)eft, (M)iddle, (R)ight
        intaking location: (M)iddle, (D)epot, (O)utpost
        (S)hoot, (L)ob
        (C)limb, (L)eave
    */

    private static Autos instance = null;

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Climber climber = Climber.getInstance();

    // PATHS AUTOS

    public Command fullTestPaths() {
        return drivetrain.followTrajectory("M_FullTest");
    }

    public Command R_MS_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarMidtake"),
            drivetrain.followTrajectory("R_CloseMidtake"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_Small_MS_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_SmallFarMidtake"),
            drivetrain.followTrajectory("R_SmallCloseMidtake"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command L_MS_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("L_FarMidtake"),
            drivetrain.followTrajectory("L_CloseMidtake"),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    public Command L_Small_MS_MS_L_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("L_SmallFarMidtake"),
            drivetrain.followTrajectory("L_SmallCloseMidtake"),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    public Command M_Depot_Outpost_Paths() {
        return drivetrain.followTrajectory("M_Depot_Outpost");
    }

    // FULL AUTOS

    public Command fullTestAuto() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            superstructure.intake.deployIntakeCommand(),
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("M_FullTest"),
            this.shootAuto(2),

            Commands.parallel(
                drivetrain.followTrajectory("M_FullTest"),
                reverseIndexAuto()
            ),
            this.shootAuto(2),

            superstructure.intake.stopIntakeCommand()
        );
    }

    public Command R_MS_MS_L() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            superstructure.intake.deployIntakeCommand(),
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_FarMidtake"),
            this.shootAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("R_CloseMidtake"),
                reverseIndexAuto()
            ),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command R_Small_MS_MS_L() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            Commands.race(
                superstructure.intake.deployIntakeCommand(),
                Commands.waitSeconds(1.5)
            ),
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("R_SmallFarMidtake"),
            this.shootAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("R_SmallCloseMidtake"),
                reverseIndexAuto()
            ),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command L_MS_MS_L() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            superstructure.intake.deployIntakeCommand(),
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("L_FarMidtake"),
            this.shootAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("L_CloseMidtake"),
                reverseIndexAuto()
            ),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    public Command L_Small_MS_MS_L() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            Commands.race(
                superstructure.intake.deployIntakeCommand(),
                Commands.waitSeconds(1.5)
            ),
            superstructure.intake.startIntakeCommand(),

            drivetrain.followTrajectory("L_SmallFarMidtake"),
            this.shootAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("L_SmallCloseMidtake"),
                reverseIndexAuto()
            ),
            this.shootAuto(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    // public Command R_MS_OS() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("R_FarMidtake"),
    //         this.shootAuto(4),

    //         drivetrain.followTrajectory("R_Outposttake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand()
    //     );
    // }

    // public Command L_MS_OS() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("L_FarMidtake"),
    //         this.shootAuto(4),

    //         drivetrain.followTrajectory("L_Outposttake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand()
    //     );
    // }

    // public Command R_MS_MS_OS() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("R_FarMidtake"),
    //         this.shootAuto(4),

    //         drivetrain.followTrajectory("R_CloseMidtake"),
    //         this.shootAuto(4),

    //         drivetrain.followTrajectory("R_Outposttake"),
    //         this.shootAuto(7),

    //         superstructure.intake.stopIntakeCommand()
    //     );
    // }

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

    private Command reverseIndexAuto() {
        return Commands.sequence(
            superstructure.index.reverseIndexCommand(),
            Commands.waitSeconds(1),
            superstructure.index.stopIndexCommand()
        );
    }

    public static Autos getInstance() {
        if (instance == null) instance = new Autos();
        return instance;
    }
}


    // public Command R_MS_L() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("R_FarMidtake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand(),
    //         drivetrain.followTrajectory("R_Leave")
    //     );
    // }

    // public Command R_Small_MS_L() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("R_SmallFarMidtake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand(),
    //         drivetrain.followTrajectory("R_Leave")
    //     );
    // }

    // public Command L_MS_L() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("L_FarMidtake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand(),
    //         drivetrain.followTrajectory("L_Leave")
    //     );
    // }

    // public Command L_Small_MS_L() {
    //     return Commands.sequence(
    //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
    //         superstructure.intake.deployIntakeCommand(),
    //         superstructure.intake.startIntakeCommand(),

    //         drivetrain.followTrajectory("L_SmallFarMidtake"),
    //         this.shootAuto(4),

    //         superstructure.intake.stopIntakeCommand(),
    //         drivetrain.followTrajectory("L_Leave")
    //     );
    // }