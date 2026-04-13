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
            drivetrain.followTrajectoryMirroredWithResetOdometry("R_FarSweep"),
            drivetrain.followTrajectoryMirrored("R_CloseSweep"),
            drivetrain.followTrajectoryMirrored("R_Leave")
        );
    }

    public Command R_TwoSweeps_Paths() {
        return Commands.sequence(
            drivetrain.followTrajectoryWithResetOdometry("R_FarSweep"),
            drivetrain.followTrajectory("R_CloseSweep"),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command M_DepotOutpost_Paths() {
        return drivetrain.followTrajectoryWithResetOdometry("M_DepotOutpost");
    }

    // FULL AUTOS

    public Command fullTestAuto() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectoryWithResetOdometry("M_FullTest"),
            this.shootRaiseHopper(2),

            Commands.parallel(
                drivetrain.followTrajectory("M_FullTest"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopper(2),

            superstructure.intake.stopIntakeCommand()
        );
    }

    public Command L_TwoSweeps() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectoryMirroredWithResetOdometry("R_FarSweep"),
            this.shootRaiseHopper(4),

            Commands.parallel(
                drivetrain.followTrajectoryMirrored("R_CloseSweep"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopper(5),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectoryMirrored("R_Leave")
        );
    }

    public Command R_TwoSweeps() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectoryWithResetOdometry("R_FarSweep"),
            this.shootRaiseHopper(4),

            Commands.parallel(
                drivetrain.followTrajectory("R_CloseSweep"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopper(5),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_Leave")
        );
    }

    public Command L_TwoSweepsBump() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectoryMirroredWithResetOdometry("R_FarSweepBump"),
            Commands.parallel(
                drivetrain.followTrajectoryMirrored("R_SOTM"),
                this.shootRaiseHopper(5)
            ),

            Commands.parallel(
                drivetrain.followTrajectoryMirrored("R_CloseSweepBump"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopper(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectoryMirrored("R_LeaveAlt")
        );
    }

    public Command R_TwoSweepsBump() {
        return Commands.sequence(
            this.initAuto(),

            drivetrain.followTrajectoryWithResetOdometry("R_FarSweepBump"),
            Commands.parallel(
                drivetrain.followTrajectory("R_SOTM"),
                this.shootRaiseHopper(5)
            ),

            Commands.parallel(
                drivetrain.followTrajectory("R_CloseSweepBump"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopper(4),

            superstructure.intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_LeaveAlt")
        );
    }

    public Command L_TwoSweepsBumpDepot() {
        return Commands.sequence(
            this.initAuto(),

            //first swipe
            drivetrain.followTrajectoryWithResetOdometry("L_TwoSweepsDepot", 0),
            Commands.parallel(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 1),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    this.shootRaiseHopper(4.5)
                )
            ),

            //second swipe intake
            Commands.parallel(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 2),
                this.reverseIndexAuto()
            ),

            //second swipe sotm to depot
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 3),
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    this.shootAuto(3.5)
                )
            ),

            //move away from depot
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 4),
                this.shootRaiseHopper(2.3)
            ),

            superstructure.intake.stopIntakeCommand()
        );
    }

    public Command M_DepotOutpost() {
        return Commands.parallel(
            drivetrain.followTrajectory("M_DepotOutpost"),
            Commands.sequence(
                this.initAuto(),
                Commands.waitSeconds(0.5),
                this.shootAuto(10),
                this.shootRaiseHopper(8)
            )
        );
    }

    // public Command L_TwoSweepsDepot() {
    //     return Commands.sequence(
    //         this.initAuto(),

    //         drivetrain.followTrajectoryMirrored("R_FarSweep"),
    //         this.shootRaiseHopper(3),

    //         Commands.parallel(
    //             drivetrain.followTrajectory("L_CloseSweepDepot", 0),
    //             this.reverseIndexAuto()
    //         ),
    //         Commands.parallel(
    //             drivetrain.followTrajectoryMirrored("R_CloseSweepDepot", 1),
    //             this.shootAuto(10)
    //         )
    //     );
    // }

    // AUTO UTILITY COMMANDS
    
    private Command initAuto() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            Commands.race(
                superstructure.intake.deployIntakeWaitCommand(),
                Commands.waitSeconds(0.7)
            ),
            superstructure.intake.startIntakeAutoCommand()
        );
    }

    private Command shootAuto(double seconds) {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.FULL_TRACKING),
            Commands.waitSeconds(seconds),
            superstructure.setStateCommand(RobotState.TURRET_TRACKING)
        );
    }

    private Command shootRaiseHopper(double seconds) {
        return Commands.parallel(
            Commands.sequence(
                superstructure.setStateCommand(RobotState.FULL_TRACKING),
                Commands.waitSeconds(seconds),
                superstructure.setStateCommand(RobotState.TURRET_TRACKING)
            ),
            Commands.sequence(
                Commands.waitSeconds(0.5),
                superstructure.intake.shootRaiseHopperCommand(),
                Commands.waitSeconds(seconds - 1.2),
                Commands.race(
                    superstructure.intake.deployIntakeWaitCommand(),
                    Commands.waitSeconds(0.7)
                ),
                superstructure.intake.startIntakeAutoCommand()
            )
        );
    }

    private Command reverseIndexAuto() {
        return Commands.sequence(
            superstructure.index.reverseIndexCommand(),
            Commands.waitSeconds(0.25),
            superstructure.index.stopIndexCommand()
        );
    }

    public static Autos getInstance() {
        if (instance == null) instance = new Autos();
        return instance;
    }
}