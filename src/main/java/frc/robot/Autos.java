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

    private static Autos instance = null;

    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Climber climber = Climber.getInstance();

    // PATHS AUTOS

    public Command fullTestPaths() {
        return drivetrain.followTrajectory("M_FullTest");
    }

    public Command L_TwoSweeps_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("L_FarSweep"),
            drivetrain.followTrajectory("L_CloseSweep"),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    public Command R_TwoSweeps_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectory("R_FarSweep"),
            drivetrain.followTrajectory("R_CloseSweep"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command M_DepotOutpost_Paths() {
        return drivetrain.followTrajectory("M_DepotOutpost");
    }

    // FULL AUTOS

    public Command fullTestAuto() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectory("M_FullTest"),
            this.shootPulsateAuto(2),

            Commands.parallel(
                drivetrain.followTrajectory("M_FullTest"),
                this.reverseIndexAuto()
            ),
            this.shootPulsateAuto(2),

            superstructure.intake.stopIntakeCommand()
        );
    }

    public Command L_TwoSweeps() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectory("L_FarSweep"),
            this.shootPulsateAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("L_CloseSweep"),
                this.reverseIndexAuto()
            ),
            this.shootPulsateAuto(5),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("L_Leave")
        );
    }

    public Command L_TwoSweepsDepot() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectory("L_FarSweep"),
            this.shootPulsateAuto(3),

            Commands.parallel(
                drivetrain.followTrajectory("L_CloseSweepDepot", 0),
                this.reverseIndexAuto()
            ),
            Commands.parallel(
                drivetrain.followTrajectory("L_CloseSweepDepot", 1),
                this.shootAuto(10)
            )
        );
    }

    public Command R_TwoSweeps() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectory("R_FarSweep"),
            this.shootPulsateAuto(4),

            Commands.parallel(
                drivetrain.followTrajectory("R_CloseSweep"),
                this.reverseIndexAuto()
            ),
            this.shootPulsateAuto(5),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command M_DepotOutpost() {
        return Commands.parallel(
            // Commands.sequence(
            //     this.initAuto(),
            //     drivetrain.followTrajectory("M_DepotOutpost")
            // ),
            // Commands.sequence(
            //     Commands.waitSeconds(2),
            //     this.shootAuto(10),
            //     this.shootPulsateAuto(8)
            // )
        );
    }

    // AUTO UTILITY COMMANDS
    
    private Command initAuto() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            Commands.race(
                superstructure.intake.deployIntakeWaitCommand(),
                Commands.waitSeconds(1)
            ),
            superstructure.intake.startIntakeCommand()
        );
    }

    private Command shootAuto(double seconds) {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.FULL_TRACKING),
            Commands.waitSeconds(seconds),
            superstructure.setStateCommand(RobotState.TURRET_TRACKING)
        );
    }

    private Command shootPulsateAuto(double seconds) {
        return Commands.parallel(
            Commands.sequence(
                superstructure.setStateCommand(RobotState.FULL_TRACKING),
                Commands.waitSeconds(seconds),
                superstructure.setStateCommand(RobotState.TURRET_TRACKING)
            ),
            Commands.sequence(
                Commands.waitSeconds(1),
                superstructure.intake.pulsateIntakeCommand(),
                Commands.waitSeconds(seconds - 2),
                superstructure.intake.deployIntakeCommand()
            )
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