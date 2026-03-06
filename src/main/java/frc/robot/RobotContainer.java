// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.RobotState;

public class RobotContainer {

    // private CommandXboxController joystick = new CommandXboxController(0);
    private LED led = LED.getInstance();

    
    private SendableChooser<Command> ledModeChooser = new SendableChooser<Command>();

    public RobotContainer() {
        configureSelector();
        configureBindings();
        // configureSysIDBindings();
    }

    public void configureSelector() {
        ledModeChooser.setDefaultOption("IDLE", led.setRobotStateCommand(RobotState.IDLE));
        ledModeChooser.addOption("MANUAL_SHOT", led.setRobotStateCommand(RobotState.MANUAL_SHOT));
        ledModeChooser.addOption("SHOOTING", led.setRobotStateCommand(RobotState.SHOOTING));
        ledModeChooser.addOption("FULL_TRACKING", led.setRobotStateCommand(RobotState.FULL_TRACKING));
        ledModeChooser.addOption("TURRET_TRACKING", led.setRobotStateCommand(RobotState.TURRET_TRACKING));
        ledModeChooser.addOption("POLICE", led.setRobotStateCommand(RobotState.POLICE));

        SmartDashboard.putData("LED Mode Chooser", ledModeChooser);
    }
    private void configureBindings() {
        // joystick.rightBumper().onTrue(Commands.runOnce(() -> led.setRobotState(LED.RobotState.MANUAL_SHOT)));
        // joystick.rightTrigger(0.05).onTrue(Commands.runOnce(() -> led.setRobotState(LED.RobotState.SHOOTING)));
        // joystick.x().onTrue(Commands.runOnce(() -> led.setRobotState(LED.RobotState.FULL_TRACKING)));
        // joystick.y().onTrue(Commands.runOnce(() -> led.setRobotState(LED.RobotState.TURRET_TRACKING)));
        // joystick.a().onTrue(Commands.runOnce(() -> led.setRobotState(LED.RobotState.IDLE)));
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
        return ledModeChooser.getSelected();
        // return Commands.none();
    }
}
