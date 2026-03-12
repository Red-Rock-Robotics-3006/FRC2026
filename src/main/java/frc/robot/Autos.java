// package frc.robot;

// // import edu.wpi.first.math.geometry.Pose2d;
// // import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;

// // import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.Superstructure;
// import frc.robot.subsystems.Superstructure.RobotState;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

// public class Autos {

//     /*Naming convention:
//         [starting position]_[intaking location][shoot or lob]_..._[climb or leave]

//         starting position: (L)eft, (C)enter, (R)ight
//         intaking location: (M)iddle, (D)epot, (O)utpost
//         (S)hoot, (L)ob
//         (C)limb, (L)eave
//     */

//     private static Autos instance = null;

//     private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
//     private final Superstructure superstructure = Superstructure.getInstance();
//     // private final Climber climber = Climber.getInstance();

//     public Command R_MS_L() {
//         return Commands.sequence(
//             superstructure.intake.startIntakeCommand(),

//             drivetrain.followTrajectory("R_FarMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(7),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             superstructure.intake.stopIntakeCommand(),
//             drivetrain.followTrajectory("R_Leave")
//         );
//     }

//     public Command R_MS_L_Paths() {
//         return Commands.sequence(
//             drivetrain.followTrajectory("R_FarMidtake"),
//             drivetrain.followTrajectory("R_Leave")
//         );
//     }

//     public Command R_MS_MS_L() {
//         return Commands.sequence(
//             superstructure.intake.startIntakeCommand(),

//             drivetrain.followTrajectory("R_FarMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(7),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             drivetrain.followTrajectory("R_CloseMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(5),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             superstructure.intake.stopIntakeCommand(),
//             drivetrain.followTrajectory("R_Leave")
//         );
//     }

//     public Command R_MS_MS_L_Paths() {
//         return Commands.sequence(
//             drivetrain.followTrajectory("R_FarMidtake"),
//             drivetrain.followTrajectory("R_CloseMidtake"),
//             drivetrain.followTrajectory("R_Leave")
//         );
//     }

//     public Command R_MS_OS() {
//         return Commands.sequence(
//             superstructure.intake.startIntakeCommand(),

//             drivetrain.followTrajectory("R_FarMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(7),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             drivetrain.followTrajectory("R_Outposttake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(5),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             superstructure.intake.stopIntakeCommand()
//         );
//     }

//     public Command R_MS_OS_Paths() {
//         return Commands.sequence(
//             drivetrain.followTrajectory("R_FarMidtake"),
//             drivetrain.followTrajectory("R_Outposttake")
//         );
//     }

//     public Command R_MS_MS_OS() {
//         return Commands.sequence(
//             superstructure.intake.startIntakeCommand(),

//             drivetrain.followTrajectory("R_FarMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(7),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             drivetrain.followTrajectory("R_CloseMidtake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(5),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             drivetrain.followTrajectory("R_Outposttake"),
//             superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(5),
//             superstructure.setStateCommand(RobotState.TURRET_TRACKING),

//             superstructure.intake.stopIntakeCommand()
//         );
//     }

//     public Command R_MS_MS_OS_Paths() {
//         return Commands.sequence(
//             drivetrain.followTrajectory("R_FarMidtake"),
//             drivetrain.followTrajectory("R_CloseMidtake"),
//             drivetrain.followTrajectory("R_Outposttake")
//         );
//     }

//     // public Command R_MS_C() {
//     //     return Commands.sequence(
//     //         superstructure.intake.startIntakeCommand(),
//     //         superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(2),
//     //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
//     //         drivetrain.followTrajectory("R_MS_C", 0),
//     //         drivetrain.followTrajectory("R_MS_C", 1),
//     //         drivetrain.followTrajectory("R_MS_C", 2),
//     //         superstructure.setStateCommand(RobotState.FULL_TRACKING).withTimeout(4),
//     //         superstructure.intake.stopIntakeCommand(),
//     //         Commands.parallel(
//     //             Commands.sequence(
//     //                 drivetrain.followTrajectory("R_MS_C", 3),
//     //                 drivetrain.pidToPoseAutoCommand(new Pose2d(3006, 3006, Rotation2d.fromDegrees(3006))) //TODO: replace with actual climbing pose this'd be very bad if ran
//     //                 ),
//     //             climber.raiseClimberCommand().withTimeout(2)
//     //         ),
//     //         climber.lowerClimberCommand().withTimeout(2),
//     //         climber.stopClimberCommand()
//     //     );
//     // }

//     public static Autos getInstance() {
//         if (instance == null) instance = new Autos();
//         return instance;
//     }
// }
