// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController driverstick = new CommandXboxController(0);
    private final CommandXboxController operatorstick = new CommandXboxController(1);

    private final double kTriggerThreshold = 0.1;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance().withController(driverstick);
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Climber climber = Climber.getInstance();
    public final LED led = LED.getInstance();

    private final Autos autos = Autos.getInstance();
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureCompSelector();
        configureCompBindings();

        configureTestSelector();
        configureTestBindings();

        // configureSysIDBindings();
    }

    private void configureCompSelector() {
        autoChooser.setDefaultOption("NO AUTO", Commands.print("good luck drivers"));

        autoChooser.addOption("Left Two Sweeps", autos.L_TwoSweeps());
        autoChooser.addOption("Left Two Sweeps Depot", autos.L_TwoSweepsDepot());
        autoChooser.addOption("Right Two Sweeps", autos.R_TwoSweeps());
        autoChooser.addOption("Middle Depot Outpost", autos.M_DepotOutpost());

        autoChooser.addOption("Full Test", autos.fullTestAuto());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    @SuppressWarnings("unused")
    private void configureTestSelector() {
        autoChooser.addOption("PATHS - Left Two Sweeps", autos.L_TwoSweeps_Paths());
        autoChooser.addOption("PATHS - Left Two Sweeps Depot", autos.L_TwoSweepsDepot());
        autoChooser.addOption("PATHS - Right Two Sweeps", autos.R_TwoSweeps_Paths());
        autoChooser.addOption("PATHS - Middle Depot Outpost", autos.M_DepotOutpost_Paths());

        autoChooser.addOption("PATHS - Full Test", autos.fullTestPaths());
    }

    private void configureCompBindings() {
        RobotModeTriggers.disabled().onTrue(superstructure.setStateCommand(RobotState.IDLE).ignoringDisable(true));
        RobotModeTriggers.teleop().onTrue(superstructure.setStateCommand(RobotState.IDLE));

        driverstick.back()
            .onTrue(drivetrain.resetHeadingCommand());

        driverstick.start()
            .onTrue(drivetrain.toggleVisionCommand());

        driverstick.leftTrigger(kTriggerThreshold)
            .onTrue(Commands.sequence(
                superstructure.intake.deployIntakeCommand(),
                superstructure.intake.startIntakeCommand()))
            .onFalse(superstructure.intake.stopIntakeCommand());

        driverstick.rightTrigger(kTriggerThreshold)
            .onTrue(superstructure.setStateCommand(RobotState.FULL_TRACKING))
            .onFalse(Commands.sequence(
                superstructure.setStateCommand(RobotState.TURRET_TRACKING),
                superstructure.intake.deployIntakeCommand()));
            
        driverstick.leftBumper() 
            .onTrue(superstructure.intake.pulsateIntakeCommand())
            .onFalse(Commands.sequence(
                superstructure.intake.stopIntakeCommand(),
                superstructure.intake.deployIntakeCommand()));

        driverstick.rightBumper() //manual hub shot
            .onTrue(superstructure.setManualShotParameterCommand())
            .onFalse(Commands.parallel(
                superstructure.setStateCommand(RobotState.IDLE),
                superstructure.intake.deployIntakeCommand()));

        // driverstick.x()
        //     .onTrue(superstructure.resetSuperStructure())
        //     .onFalse(Commands.runOnce(() -> drivetrain.disableIgnoreCameraDistance(), drivetrain));

        driverstick.y()
            .onTrue(superstructure.setStateCommand(RobotState.REVERSE))
            .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));
            
        // driverstick.a()
        //     .onTrue(superstructure.intake.reverseIntakeCommand())
        //     .onFalse(superstructure.intake.stopIntakeCommand());

        // driverstick.b()
        //     .onTrue(superstructure.setStateCommand(RobotState.IDLE));

        // driverstick.povUp() //left paddle
        //     .onTrue(Commands.sequence(
        //         superstructure.intake.pushRetractIntakeCommand(),
        //         climber.raiseClimberCommand()))
        //     .onFalse(climber.stopClimberCommand());

        // driverstick.povDown() //right paddle
        //     .onTrue(Commands.sequence(
        //         superstructure.intake.pushRetractIntakeCommand(),
        //         climber.lowerClimberCommand()))
        //     .onFalse(climber.stopClimberCommand());

        driverstick.povLeft()
            .onTrue(superstructure.intake.pushRetractIntakeCommand());
            
        driverstick.povRight()
            .onTrue(superstructure.intake.resetIntakeExtensionCommand());

        operatorstick.y()
            .onTrue(superstructure.increaseAutoAimOffsetCommand());

        operatorstick.a()
            .onTrue(superstructure.decreaseAutoAimOffsetCommand());

        operatorstick.x()
            .onTrue(Commands.runOnce(() -> drivetrain.enableIgnoreCameraDistance(), drivetrain))
            .onFalse(Commands.runOnce(() -> drivetrain.disableIgnoreCameraDistance(), drivetrain));

        operatorstick.b()
            .onTrue(superstructure.toggleTrenchSafetyCommand());

        operatorstick.povLeft()
            .onTrue(superstructure.intake.deployIntakeCommand());
    }

    @SuppressWarnings("unused")
    private void configureTestBindings() {
        driverstick.x() //manual lerp tuning shot
            .onTrue(superstructure.setLerpTuneParameterCommand())
            .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // driverstick.povLeft() //definitely delete this for comp lol (it'll look cool during practice tho)
        //     .onTrue(led.togglePoliceCommand());

        // driverstick.a()
        //     .onTrue(Commands.runOnce(() -> superstructure.intake.pushRetractIntake()));

        driverstick.a()
            .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningPosA()));

        // driverstick.b()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningPosB()));

        driverstick.b()
            .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningRotation()));
    }

    // private void configureSysIDBindings() {
    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     driverstick.back().and(driverstick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     driverstick.back().and(driverstick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     driverstick.start().and(driverstick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     driverstick.start().and(driverstick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
