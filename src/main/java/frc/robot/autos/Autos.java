package frc.robot.autos;

import choreo.Choreo;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
// import choreo.util.ChoreoAllianceFlipUtil;
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

    public RedRockAuto noAuto = new RedRockAuto("NO AUTO",
        Commands.print("good luck drivers"),
        new Pose2d(),
        new Pose2d()
    );

    public RedRockAuto fullTest = new RedRockAuto("Full Test", 
        Commands.sequence(
            Commands.parallel(
                this.initAuto(),
                drivetrain.followTrajectory("M_FullTest")
            ),
            this.shootRaiseHopperAuto(2),

            Commands.parallel(
                drivetrain.followTrajectory("M_FullTest2"),
                this.reverseIndexAuto()
            ),
            this.shootRaiseHopperAuto(2),

            superstructure.intake.stopIntakeCommand()
        ),
        
        Choreo.loadTrajectory("M_FullTest")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("M_FullTest")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto L_TwoSweepsDepot = new RedRockAuto("Left - Two Sweeps Depot",
        Commands.sequence(
            //first swipe intake
            Commands.parallel(
                this.initAuto(),
                drivetrain.followTrajectory("L_TwoSweepsDepot", 0)
            ),

            //first swipe sotm to trench
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 1),
                Commands.sequence(
                    Commands.waitSeconds(0.15),
                    this.shootRaiseHopperAuto(4.35)
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
                    Commands.waitSeconds(0.15),
                    this.shootAuto(3.7)
                )
            ),

            //move away from depot
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweepsDepot", 4),
                this.shootRaiseHopperAuto(2.05)
            ),

            superstructure.intake.stopIntakeCommand()
        ),

        Choreo.loadTrajectory("L_TwoSweepsDepot")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("L_TwoSweepsDepot")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto L_TwoSweeps = new RedRockAuto("Left - Two Sweeps",
        Commands.sequence(
            //first swipe intake
            Commands.parallel(
                this.initAuto(),
                drivetrain.followTrajectory("L_TwoSweeps", 0)
            ),

            //first swipe sotm to trench
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweeps", 1),
                Commands.sequence(
                    Commands.waitSeconds(0.15),
                    this.shootRaiseHopperAuto(4.35)
                )
            ),

            //second swipe intake
            Commands.parallel(
                drivetrain.followTrajectory("L_TwoSweeps", 2),
                this.reverseIndexAuto()
            ),

            //second swipe sotm to trench
            Commands.deadline(
                drivetrain.followTrajectory("L_TwoSweeps", 3),
                this.shootRaiseHopperAuto(5.6)
            ),

            superstructure.intake.stopIntakeCommand()
        ),

        Choreo.loadTrajectory("L_TwoSweeps")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("L_TwoSweeps")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto L_Follow = new RedRockAuto("Left - Follow",
        Commands.sequence(
            this.initAuto(),
            this.shootRaiseHopperAuto(3),

            //first swipe intake
            drivetrain.followTrajectory("L_Follow", 0),

            Commands.waitSeconds(0.15),
            this.shootRaiseHopperAuto(6.5),

            drivetrain.followTrajectory("L_Follow", 1)
        ),

        Choreo.loadTrajectory("L_Follow")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("L_Follow")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto L_Leave = new RedRockAuto("Left - Leave",
        Commands.sequence(
            this.initAuto(),
            this.shootRaiseHopperAuto(3),

            drivetrain.followTrajectory("L_Leave"),
            superstructure.intake.stopIntakeCommand()
        ),

        Choreo.loadTrajectory("L_Leave")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("L_Leave")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto L_DepotLeave = new RedRockAuto("Left - Depot Leave",
        Commands.sequence(
            //first swipe intake
            Commands.parallel(
                this.initAuto(),
                drivetrain.followTrajectory("M_DepotLeave", 0)
            ),

            this.shootRaiseHopperAuto(7.5),
            //first swipe sotm to trench
            drivetrain.followTrajectory("M_DepotLeave", 1)
        ),

        Choreo.loadTrajectory("M_DepotLeave")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("M_DepotLeave")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto R_TwoSweeps = new RedRockAuto("Right - Two Sweeps",
        Commands.sequence(
            //first swipe intake
            Commands.parallel(
                this.initAuto(),
                drivetrain.followTrajectory("R_TwoSweeps", 0)
            ),

            //first swipe sotm to trench
            Commands.deadline(
                drivetrain.followTrajectory("R_TwoSweeps", 1),
                Commands.sequence(
                    Commands.waitSeconds(0.15),
                    this.shootRaiseHopperAuto(4.35)
                )
            ),

            //second swipe intake
            Commands.parallel(
                drivetrain.followTrajectory("R_TwoSweeps", 2),
                this.reverseIndexAuto()
            ),

            //second swipe sotm to trench
            Commands.deadline(
                drivetrain.followTrajectory("R_TwoSweeps", 3),
                this.shootRaiseHopperAuto(5.6)
            ),

            superstructure.intake.stopIntakeCommand()
        ),

        Choreo.loadTrajectory("R_TwoSweeps")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("R_TwoSweeps")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto R_Follow = new RedRockAuto("Right - Follow",
        Commands.sequence(
            this.initAuto(),
            this.shootRaiseHopperAuto(3),

            //first swipe intake
            drivetrain.followTrajectory("R_Follow", 0),

            Commands.waitSeconds(0.15),
            this.shootRaiseHopperAuto(6.5),

            drivetrain.followTrajectory("R_Follow", 1)
        ),

        Choreo.loadTrajectory("R_Follow")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("R_Follow")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto M_Depot = new RedRockAuto("Middle - Depot",
        Commands.parallel(
            drivetrain.followTrajectory("M_Depot"),
            Commands.sequence(
                this.initAuto(),
                Commands.waitSeconds(0.5),
                this.shootAuto(7),
                this.shootRaiseHopperAuto(8)
            )
        ),

        Choreo.loadTrajectory("M_Depot")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("M_Depot")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    public RedRockAuto M_Preload = new RedRockAuto("Middle - Preload",
        Commands.parallel(
            drivetrain.followTrajectory("M_Preload"),
            Commands.sequence(
                this.initAuto(),
                Commands.waitSeconds(0.5),
                this.shootRaiseHopperAuto(5)
            )
        ),

        Choreo.loadTrajectory("M_Preload")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue())).get(),
        
        Choreo.loadTrajectory("M_Preload")
            .flatMap(traj -> traj.getInitialPose(!drivetrain.isBlue()))
            .map(ChoreoAllianceFlipUtil.getFlipper()::flip).get()
    );

    // AUTO UTILITY

    public void resetPoseForAuto(RedRockAuto auto) {
        if (DriverStation.isDisabled()) {
            drivetrain.resetPose(
                drivetrain.isBlue() ?
                    auto.getBlueInitialPose() :
                    auto.getRedInitialPose());
        }
    }
    
    private Command initAuto() {
        return Commands.sequence(
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            superstructure.intake.deployIntakeCommand(),
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

    private Command shootRaiseHopperAuto(double seconds) {
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
            Commands.waitSeconds(0.35),
            superstructure.index.stopIndexCommand()
        );
    }

    public static Autos getInstance() {
        if (instance == null) instance = new Autos();
        return instance;
    }
}