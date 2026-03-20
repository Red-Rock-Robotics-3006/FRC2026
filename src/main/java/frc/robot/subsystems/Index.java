package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Index extends SubsystemBase {
    private static Index instance = null;

    private RedRockTalon dyeRotorMotor = new RedRockTalon(31, "index-dyerotor-motor", "*");
    // private RedRockTalon kickerMotor = new RedRockTalon(32, "index-kicker-motor", "*");

    private Slot1Configs slot1Configs = new Slot1Configs();

    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("index/index speed", 1).withTuningEnabled(true); //SHOULD BE RPM FOR MM VELOCITY VOLTAGE
    private SmartDashboardNumber indexReverseSpeed = new SmartDashboardNumber("index/index reverse speed", -0.3).withTuningEnabled(true);

    // private SmartDashboardNumber kickerSpeed = new SmartDashboardNumber("index/kicker speed", 0.6).withTuningEnabled(true);

    private final double dyeRotorGearRatio = 45.82608144444; // 25 * 44 / 24;
    private SmartDashboardNumber indexSafePosition = new SmartDashboardNumber("index/index safe position", 0).withTuningEnabled(true);
    private SmartDashboardNumber indexSafePositionTolerance = new SmartDashboardNumber("index/index safe position tolerance", 5).withTuningEnabled(true);

    private Index() {
        super();

        this.dyeRotorMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(5)
            .withKI(0)
            .withKD(0)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(450)
            .withMotionMagicCruiseVelocity(99999)
            .withMotionMagicJerk(250)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);

        // this.kickerMotor.withMotorOutputConfigs(
        //     new MotorOutputConfigs()
        //     .withInverted(InvertedValue.Clockwise_Positive)
        //     .withPeakForwardDutyCycle(1d)
        //     .withPeakReverseDutyCycle(-1d)
        //     .withNeutralMode(NeutralModeValue.Brake)
        // ).withCurrentLimitConfigs(
        //     new CurrentLimitsConfigs()
        //     .withSupplyCurrentLimit(45)
        //     .withSupplyCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(80)
        //     .withStatorCurrentLimitEnable(true)
        // ).withTuningEnabled(true);

        slot1Configs
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0);

        this.dyeRotorMotor.motor.getConfigurator().apply(slot1Configs);

        this.dyeRotorMotor.resetMotor();
    }

    private void setIndexSpeed(double speed) {
        this.dyeRotorMotor.motor.setControl(new DutyCycleOut(speed).withEnableFOC(false));
    }

    public void startIndex() {
        // this.setIndexSpeed(indexSpeed.getNumber());
        // this.setKickerSpeed(kickerSpeed.getNumber());
        this.setIndexSpeed(indexSpeed.getNumber());

    }

    public void reverseIndex() {
        this.setIndexSpeed(indexReverseSpeed.getNumber());
    }

    public void stopIndex() {
        this.dyeRotorMotor.motor.setControl(new NeutralOut());
        // this.setKickerSpeed(0);
    }

    // private void setKickerSpeed(double speed) {
    //     this.kickerMotor.motor.setControl(new DutyCycleOut(speed));
    // }

    // public void startKicker() {
    //     this.setKickerSpeed(kickerSpeed.getNumber());
    // }

    // public void stopKicker() {
    //     this.setKickerSpeed(0);
    // }

    public void khangaiIsAChud() {
        this.dyeRotorMotor.motor.setControl(new CoastOut());
        this.dyeRotorMotor.motor.setPosition(this.dyeRotorMotor.motor.getPosition().getValueAsDouble() % dyeRotorGearRatio);
        this.dyeRotorMotor.motor.setControl(new MotionMagicVoltage(indexSafePosition.getNumber()).withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void indexChudTuningPosition() {
        this.dyeRotorMotor.motor.setPosition(this.dyeRotorMotor.motor.getPosition().getValueAsDouble() % dyeRotorGearRatio);
    }

    public void indexChudTuning1() {
        this.dyeRotorMotor.motor.setPosition(this.dyeRotorMotor.motor.getPosition().getValueAsDouble() % dyeRotorGearRatio);
        this.dyeRotorMotor.motor.setControl(new MotionMagicVoltage(indexSafePosition.getNumber()).withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public void indexChudTuning2() {
        this.dyeRotorMotor.motor.setPosition(this.dyeRotorMotor.motor.getPosition().getValueAsDouble() % dyeRotorGearRatio);
        this.dyeRotorMotor.motor.setControl(new MotionMagicVoltage(indexSafePosition.getNumber()+20).withSlot(0).withEnableFOC(true).withOverrideBrakeDurNeutral(true));
    }

    public Command indexChudTuningPositionCommand() {
        return Commands.runOnce(() -> this.indexChudTuningPosition());
    }

    public Command indexChudTuning1Command() {
        return Commands.runOnce(() -> this.indexChudTuning1());
    }

    public Command indexChudTuning2Command() {
        return Commands.runOnce(() -> this.indexChudTuning2());
    }

    public boolean inSafePosition() {
        return Math.abs(this.dyeRotorMotor.motor.getPosition().getValueAsDouble() - this.indexSafePosition.getNumber())
            < this.indexSafePositionTolerance.getNumber();
    }

    public Command khangaiIsAChudCommand() {
        return Commands.runOnce(() -> this.khangaiIsAChud(), this);
    }

    public Command startIndexCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.startIndex(), this)//,
            // Commands.runOnce(() -> this.startKicker())
        );
    }

    public Command reverseIndexCommand() {
        return Commands.runOnce(() -> this.reverseIndex(), this);
    }

    public Command stopIndexCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.stopIndex(), this)//,
            // Commands.runOnce(() -> this.stopKicker())
        );
    }

    @Override
    public void periodic() {
        this.dyeRotorMotor.update();
    }

    public static Index getInstance() {
        if (instance == null) {
            instance = new Index();
        }
        return instance;
    }
}
