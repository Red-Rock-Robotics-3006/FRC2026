package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Index extends SubsystemBase {
    private static Index instance = null;

    // private RedRockTalon conveyorMotor = new RedRockTalon(0, "index-conveyor", "*"); //TODO
    // private RedRockTalon centeringMotor = new RedRockTalon(0, "index-centering", "*"); //TODO
    private RedRockTalon feedMotor = new RedRockTalon(41, "index-feed", "*"); //TODO

    // private SmartDashboardNumber conveyorSpeed = new SmartDashboardNumber("index/conveyor speed", 0.2).withTuningEnabled(true);
    // private SmartDashboardNumber conveyorReverseSpeed = new SmartDashboardNumber("index/conveyor reverse speed", -0.2).withTuningEnabled(true);    
    // private SmartDashboardNumber centeringSpeed = new SmartDashboardNumber("index/centering speed", 0.2).withTuningEnabled(true);
    // private SmartDashboardNumber centeringReverseSpeed = new SmartDashboardNumber("index/centering reverse speed", -0.2).withTuningEnabled(true);    
    private SmartDashboardNumber feedSpeed = new SmartDashboardNumber("index/feed speed", 0.8).withTuningEnabled(true);
    private SmartDashboardNumber feedReverseSpeed = new SmartDashboardNumber("index/feed reverse speed", -0.6).withTuningEnabled(true);    

    private Index() {
        super();

        // this.conveyorMotor.withMotorOutputConfigs(
        //     new MotorOutputConfigs()
        //     .withInverted(InvertedValue.CounterClockwise_Positive)
        //     .withPeakForwardDutyCycle(1d)
        //     .withPeakReverseDutyCycle(-1d)
        //     .withNeutralMode(NeutralModeValue.Brake)
        // )
        // .withCurrentLimitConfigs(
        //     new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(45)
        //     .withSupplyCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(80)
        //     .withStatorCurrentLimitEnable(true)
        // ).withTuningEnabled(true);

        // this.centeringMotor.withMotorOutputConfigs(
        //     new MotorOutputConfigs()
        //     .withInverted(InvertedValue.CounterClockwise_Positive)
        //     .withPeakForwardDutyCycle(1d)
        //     .withPeakReverseDutyCycle(-1d)
        //     .withNeutralMode(NeutralModeValue.Brake)
        // )
        // .withCurrentLimitConfigs(
        //     new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(45)
        //     .withSupplyCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(80)
        //     .withStatorCurrentLimitEnable(true)
        // ).withTuningEnabled(true);

        this.feedMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
    }

    // private void startConveyor() {
    //     this.conveyorMotor.motor.setControl(new DutyCycleOut(conveyorSpeed.getNumber()));
    // }
    // private void reverseConveyor() {
    //     this.conveyorMotor.motor.setControl(new DutyCycleOut(conveyorReverseSpeed.getNumber()));
    // }
    // private void stopConveyor() {
    //     this.conveyorMotor.motor.setControl(new DutyCycleOut(0d));
    // }

    // private void startCentering() {
    //     this.centeringMotor.motor.setControl(new DutyCycleOut(centeringSpeed.getNumber()));
    // }
    // private void reverseCentering() {
    //     this.centeringMotor.motor.setControl(new DutyCycleOut(centeringReverseSpeed.getNumber()));
    // }
    // private void stopCentering() {
    //     this.centeringMotor.motor.setControl(new DutyCycleOut(0d));
    // }

    public void startFeed() {
        this.feedMotor.motor.setControl(new DutyCycleOut(feedSpeed.getNumber()));
    }
    private void reverseFeed() {
        this.feedMotor.motor.setControl(new DutyCycleOut(feedReverseSpeed.getNumber()));
    }
    public void stopFeed() {
        this.feedMotor.motor.setControl(new DutyCycleOut(0d));
    }

    // public void startIndex() {
    //     startConveyor();
    //     startCentering();
    //     startFeed();
    // }

    // public void stopIndex() {
    //     stopConveyor();
    //     stopCentering();
    //     stopFeed();
    // }

    // public Command startConveyorCommand() {
    //     return Commands.runOnce(() -> startConveyor(), this);
    // }
    // public Command reverseConveyorCommand() {
    //     return Commands.runOnce(() -> reverseConveyor(), this);
    // }
    // public Command stopConveyorCommand() {
    //     return Commands.runOnce(() -> stopConveyor(), this);
    // }

    // public Command startCenteringCommand() {
    //     return Commands.runOnce(() -> startCentering(), this);
    // }
    // public Command reverseCenteringCommand() {
    //     return Commands.runOnce(() -> reverseCentering(), this);
    // }
    // public Command stopCenteringCommand() {
    //     return Commands.runOnce(() -> stopCentering(), this);
    // }

    public Command startFeedCommand() {
        return Commands.runOnce(() -> startFeed(), this);
    }
    public Command reverseFeedCommand() {
        return Commands.runOnce(() -> reverseFeed(), this);
    }
    public Command stopFeedCommand() {
        return Commands.runOnce(() -> stopFeed(), this);
    }

    // public Command startIndexCommand() {
    //     return Commands.sequence(
    //         startConveyorCommand(),
    //         startCenteringCommand(),
    //         startFeedCommand()
    //     );
    // }

    // public Command stopIndexCommand() {
    //     return Commands.sequence(
    //         stopConveyorCommand(),
    //         stopCenteringCommand(),
    //         stopFeedCommand()
    //     );
    // }

    // public Command reverseIndexCommand() {
    //     return Commands.sequence(
    //         reverseConveyorCommand(),
    //         reverseCenteringCommand(),
    //         reverseFeedCommand()
    //     );
    // }

    @Override
    public void periodic() {
        // this.conveyorMotor.update();
        // this.centeringMotor.update();
        this.feedMotor.update();
    }

    public static Index getInstance() {
        if (instance == null) {
            instance = new Index();
        }
        return instance;
    }
}