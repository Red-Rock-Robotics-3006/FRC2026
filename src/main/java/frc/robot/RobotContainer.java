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
import frc.robot.autos.Autos;
import frc.robot.autos.RedRockAuto;
// import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class RobotContainer {
    private final CommandXboxController driverstick = new CommandXboxController(0);
    private final CommandXboxController operatorstick = new CommandXboxController(1);

    public static final double kTriggerThreshold = 0.1;

    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance().withController(driverstick);
    private final Superstructure superstructure = Superstructure.getInstance();
    // private final Climber climber = Climber.getInstance();
    public final LED led = LED.getInstance();

    private final Autos autos = Autos.getInstance();
    private SendableChooser<RedRockAuto> autoChooser = new SendableChooser<RedRockAuto>();

    public RobotContainer() {
        configureCompSelector();
        configureCompBindings();

        // configureTestBindings();
        // configureSysIDBindings();
    }

    private void configureCompSelector() {
        autoChooser.setDefaultOption(autos.noAuto.getName(), autos.noAuto);

        autoChooser.addOption(autos.L_TwoSweeps.getName(), autos.L_TwoSweeps);
        autoChooser.addOption(autos.L_TwoSweepsDepot.getName(), autos.L_TwoSweepsDepot);
        
        autoChooser.addOption(autos.R_TwoSweeps.getName(), autos.R_TwoSweeps);

        autoChooser.addOption(autos.M_Depot.getName(), autos.M_Depot);
        autoChooser.addOption(autos.M_Preload.getName(), autos.M_Preload);
        
        autoChooser.addOption(autos.fullTest.getName(), autos.fullTest);

        autoChooser.onChange(autos::resetPoseForAuto);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureCompBindings() {
        RobotModeTriggers.disabled().onTrue(superstructure.setStateCommand(RobotState.IDLE).ignoringDisable(true));
        RobotModeTriggers.teleop().onTrue(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

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
                superstructure.intake.deployIntakeStoplessCommand()));
            
        driverstick.leftBumper() 
            .onTrue(superstructure.intake.shootRaiseHopperCommand())
            .onFalse(superstructure.intake.deployIntakeCommand());

        driverstick.rightBumper() //manual hub shot
            .onTrue(superstructure.setManualShotParameterCommand())
            .onFalse(Commands.parallel(
                superstructure.setStateCommand(RobotState.IDLE),
                superstructure.intake.deployIntakeStoplessCommand()));

        driverstick.x()
            .onTrue(Commands.runOnce(() -> superstructure.turret.calibrateTurret()));

        driverstick.y()
            .onTrue(superstructure.setStateCommand(RobotState.REVERSE))
            .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));
            
        driverstick.a()
            .onTrue(superstructure.intake.reverseIntakeCommand())
            .onFalse(superstructure.intake.stopIntakeCommand());

        driverstick.b()
            .onTrue(superstructure.intake.resetIntakeExtensionCommand());

        operatorstick.y()
            .onTrue(superstructure.increaseAutoAimRPMOffsetCommand());

        operatorstick.a()
            .onTrue(superstructure.decreaseAutoAimRPMOffsetCommand());

        operatorstick.b()
            .onTrue(superstructure.toggleTrenchSafetyCommand());

        operatorstick.povLeft()
            .onTrue(superstructure.intake.deployIntakeCommand());

        operatorstick.povUp()
            .onTrue(superstructure.increaseAutoAimHoodOffsetCommand());

        operatorstick.povDown()
            .onTrue(superstructure.decreaseAutoAimHoodOffsetCommand());

        operatorstick.leftStick()
            .onTrue(led.togglePoliceCommand());

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
    }

    @SuppressWarnings("unused")
    private void configureTestBindings() {
        // driverstick.x() //manual lerp tuning shot
        //     .onTrue(superstructure.setLerpTuneParameterCommand())
        //     .onFalse(superstructure.setStateCommand(RobotState.TURRET_TRACKING));

        // driverstick.a()
        //     .onTrue(Commands.runOnce(() -> superstructure.intake.pushRetractIntake()));

        // driverstick.a()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningPosA()));

        // driverstick.b()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningPosB()));

        // driverstick.b()
        //     .onTrue(Commands.runOnce(() -> superstructure.turret.setTuningRotation()));
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
        return autoChooser.getSelected().getCommand();
    }
}
