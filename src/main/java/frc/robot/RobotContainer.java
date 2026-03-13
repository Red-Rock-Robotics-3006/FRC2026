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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.shooter.autoaim.EditableShotParameter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController joystick = new CommandXboxController(0);

    private final double kTriggerThreshold = 0.1;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance().withController(joystick);
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Climber climber = Climber.getInstance();
    // public final LED led = LED.getInstance();

    // private final Intake intake = Intake.getInstance();

    // private final Autos autos = Autos.getInstance();
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private EditableShotParameter lerpingShotParameter = new EditableShotParameter(20, 3000, "lerping shot parameter");
    private EditableShotParameter hubShotParameter = new EditableShotParameter(30, 3000, "hub shot parameter");

    public RobotContainer() {
        configureSelector();
        configureBindings();
        // configureSysIDBindings();
    }

    public void configureSelector() {
        autoChooser.setDefaultOption("NO AUTO", Commands.print("good luck drivers"));

        // REAL AUTOS

        // autoChooser.addOption("Right Midtake Leave", autos.R_MS_L());
        // autoChooser.addOption("Right Two Midtakes Leave", autos.R_MS_MS_L());
        // autoChooser.addOption("Right Midtake Outpost", autos.R_MS_OS());
        // autoChooser.addOption("Right Two Midtakes Outpost", autos.R_MS_MS_OS());

        // // PATHS AUTOS

        // autoChooser.addOption("PATHS - Right Midtake Leave", autos.R_MS_L_Paths());
        // autoChooser.addOption("PATHS - Right Two Midtakes Leave", autos.R_MS_MS_L_Paths());
        // autoChooser.addOption("PATHS - Right Midtake Outpost", autos.R_MS_OS_Paths());
        // autoChooser.addOption("PATHS - Right Two Midtakes Outpost", autos.R_MS_MS_OS_Paths());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // RobotModeTriggers.disabled().onTrue(superstructure.setStateCommand(RobotState.IDLE).ignoringDisable(true));
        // RobotModeTriggers.teleop().onTrue(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // joystick.back().onTrue(drivetrain.resetHeadingCommand());

        joystick.leftTrigger(kTriggerThreshold)
            .onTrue(superstructure.intake.startIntakeCommand())
            .onFalse(superstructure.intake.stopIntakeCommand());

        // joystick.rightTrigger(kTriggerThreshold)
        //     .onTrue(superstructure.setStateCommand(RobotState.FULL_TRACKING))
        //     .onFalse(Commands.parallel(
        //         superstructure.setStateCommand(RobotState.TURRET_TRACKING),
        //         superstructure.intake.deployIntakeCommand()));
            
        joystick.leftBumper() 
            .onTrue(superstructure.intake.pulsateIntakeCommand())
            .onFalse(superstructure.intake.deployIntakeCommand());

        joystick.a()
            .onTrue(superstructure.intake.deployIntakeCommand());

        joystick.x()
            .onTrue(superstructure.intake.pushRetractIntakeCommand());

        joystick.b()
            .onTrue(superstructure.intake.resetIntakeExtensionCommand());

        joystick.rightBumper() //manual hub shot
            .onTrue(superstructure.setManualShotParameterCommand())
            .onFalse(Commands.parallel(
                superstructure.setStateCommand(RobotState.IDLE)));
                // superstructure.intake.deployIntakeCommand()));
            
        // joystick.a()
        //     .onTrue(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // joystick.b()
        //     .onTrue(superstructure.setStateCommand(RobotState.IDLE));
        
        // joystick.x() //manual lerp tuning shot
        //     .onTrue(superstructure.setLerpTuneShotParameterCommand(lerpingShotParameter))
        //     .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // joystick.y()
        //     .onTrue(superstructure.intakeSafeStowCommand());

        // joystick.povRight()
        //     .onTrue(superstructure.resetShooterHoodCommand());

        // joystick.povUp() //left paddle
        //     .onTrue(Commands.sequence(
        //         // superstructure.setStateCommand(RobotState.IDLE),
        //         // superstructure.intakeSafeStowCommand(),
        //         climber.raiseClimberCommand()))
        //     .onFalse(climber.stopClimberCommand());

        // joystick.povDown() //right paddle
        //     .onTrue(Commands.sequence(
        //         // superstructure.intakeSafeStowCommand(),
        //         climber.lowerClimberCommand()))
        //     .onFalse(climber.stopClimberCommand());

        // joystick.povLeft() //definitely delete this for comp lol (it'll look cool during practice tho)
        //     .onTrue(led.togglePoliceCommand());
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
