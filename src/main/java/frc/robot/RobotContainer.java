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

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final double kTriggerThreshold = 0.1;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance().withController(joystick);
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Climber climber = Climber.getInstance();
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

        autoChooser.addOption("Right Midtake Leave", autos.R_MS_L());
        autoChooser.addOption("Right Two Midtakes Leave", autos.R_MS_MS_L());
        autoChooser.addOption("Right Midtake Outpost", autos.R_MS_OS());
        autoChooser.addOption("Right Two Midtakes Outpost", autos.R_MS_MS_OS());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureTestSelector() {
        autoChooser.addOption("PATHS - Right Midtake Leave", autos.R_MS_L_Paths());
        autoChooser.addOption("PATHS - Right Two Midtakes Leave", autos.R_MS_MS_L_Paths());
        autoChooser.addOption("PATHS - Right Midtake Outpost", autos.R_MS_OS_Paths());
        autoChooser.addOption("PATHS - Right Two Midtakes Outpost", autos.R_MS_MS_OS_Paths());
    }

    private void configureCompBindings() {
        RobotModeTriggers.disabled().onTrue(superstructure.setStateCommand(RobotState.IDLE).ignoringDisable(true));
        RobotModeTriggers.teleop().onTrue(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        joystick.back().onTrue(drivetrain.resetHeadingCommand());

        joystick.leftTrigger(kTriggerThreshold)
            .onTrue(Commands.sequence(
                superstructure.intake.deployIntakeCommand(),
                superstructure.intake.startIntakeCommand()))
            .onFalse(superstructure.intake.stopIntakeCommand());

        joystick.rightTrigger(kTriggerThreshold)
            .onTrue(superstructure.setStateCommand(RobotState.FULL_TRACKING))
            .onFalse(Commands.parallel(
                superstructure.setStateCommand(RobotState.TURRET_TRACKING),
                superstructure.intake.deployIntakeCommand()));
            
        joystick.leftBumper() 
            .onTrue(superstructure.intake.pulsateIntakeCommand())
            .onFalse(Commands.sequence(
                superstructure.intake.stopIntakeCommand(),
                superstructure.intake.deployIntakeCommand()));

        joystick.rightBumper() //manual hub shot
            .onTrue(superstructure.setManualShotParameterCommand())
            .onFalse(Commands.parallel(
                superstructure.setStateCommand(RobotState.TURRET_TRACKING),
                superstructure.intake.deployIntakeCommand()));
            
        joystick.a()
            .onTrue(superstructure.intake.reverseIntakeCommand())
            .onFalse(superstructure.intake.stopIntakeCommand());

        joystick.b()
            .onTrue(superstructure.setStateCommand(RobotState.IDLE));

        joystick.povUp() //left paddle
            .onTrue(Commands.sequence(
                superstructure.setStateCommand(RobotState.IDLE),
                // superstructure.intakeSafeStowCommand(),
                superstructure.intake.pushRetractIntakeCommand(),
                climber.raiseClimberCommand()))
            .onFalse(climber.stopClimberCommand());

        joystick.povDown() //right paddle
            .onTrue(Commands.sequence(
                // superstructure.intakeSafeStowCommand(),
                superstructure.intake.pushRetractIntakeCommand(),
                climber.lowerClimberCommand()))
            .onFalse(climber.stopClimberCommand());
    }

    private void configureTestBindings() {
        joystick.x() //manual lerp tuning shot
            .onTrue(superstructure.setLerpTuneParameterCommand())
            .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // joystick.povLeft() //definitely delete this for comp lol (it'll look cool during practice tho)
        //     .onTrue(led.togglePoliceCommand());

        // joystick.a()
        //     .onTrue(Commands.runOnce(() -> superstructure.intake.pushRetractIntake()));

        joystick.povRight()
            .onTrue(superstructure.intake.resetIntakeExtensionCommand());

        joystick.y()
            .onTrue(superstructure.index.reverseIndexCommand())
            .onFalse(superstructure.index.stopIndexCommand());

        joystick.start()
            .onTrue(Commands.runOnce(() -> superstructure.drivetrain.enableIgnoreCamera(), superstructure.drivetrain))
            .onFalse(Commands.runOnce(() -> superstructure.drivetrain.disableIgnoreCamera(), superstructure.drivetrain));

        // joystick.b()
        //     .onTrue(superstructure.index.indexChudTuningPositionCommand());

        // joystick.y()
        //     .onTrue(superstructure.index.indexChudTuning1Command());

        // joystick.a()
        //     .onTrue(superstructure.index.indexChudTuning2Command());

        // joystick.a()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningPosA(), superstructure.turret));
        
        // joystick.y()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningRotation(), superstructure.turret));

        joystick.povLeft()
            .onTrue(Commands.runOnce(() -> superstructure.turret.calibrateTurret(), superstructure.turret));

        // joystick.
    }

    // private void configureSysIDBindings() {
    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //     joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //     joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //     joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    // }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
