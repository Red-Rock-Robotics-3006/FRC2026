package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Superstructure.RobotState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
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
    private final Intake intake = Intake.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Climber climber = Climber.getInstance();

    public Command R_MS_L() {
        return Commands.sequence(
            intake.startIntakeCommand().withTimeout(0.5),
            superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(2),
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            drivetrain.followTrajectory("R_MS_L", 0),
            drivetrain.followTrajectory("R_MS_L", 1),
            drivetrain.followTrajectory("R_MS_L", 2),
            superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(4),
            intake.stopIntakeCommand(),
            drivetrain.followTrajectory("R_MS_L", 3)
        );
    }

    public Command R_MS_C() {
        return Commands.sequence(
            intake.startIntakeCommand().withTimeout(0.5),
            superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(2),
            superstructure.setStateCommand(RobotState.TURRET_TRACKING),
            drivetrain.followTrajectory("R_MS_C", 0),
            drivetrain.followTrajectory("R_MS_C", 1),
            drivetrain.followTrajectory("R_MS_C", 2),
            superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(4),
            intake.stopIntakeCommand(),
            Commands.parallel(
                Commands.sequence(
                    drivetrain.followTrajectory("R_MS_C", 3),
                    Commands.print("drivetrain pid to pose")
                ),
                climber.raiseClimberCommand().withTimeout(2)
            ),
            climber.lowerClimberCommand().withTimeout(2),
            climber.stopClimberCommand()
        );
    }

    public static Autos getInstance() {
        if (instance == null) instance = new Autos();
        return instance;
    }
}
