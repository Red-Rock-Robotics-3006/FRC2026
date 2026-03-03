package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Index extends SubsystemBase {
    private static Index instance = null;

    private RedRockTalon indexMotor = new RedRockTalon(31, "index-motor", "*");
    private Slot1Configs slot1Configs = new Slot1Configs();

    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("index/index speed", 4500).withTuningEnabled(true);
    private SmartDashboardNumber indexReverseSpeed = new SmartDashboardNumber("index/index reverse speed", -3000).withTuningEnabled(true);

    private final double dyeRotorGearRatio = 25 * 44 / 24;
    private SmartDashboardNumber indexSafePosition = new SmartDashboardNumber("index/index safe position", 0).withTuningEnabled(true);
    private SmartDashboardNumber indexSafePositionTolerance = new SmartDashboardNumber("index/index safe position tolerance", 5).withTuningEnabled(true);

    private Index() {
        super();

        this.indexMotor.withMotorOutputConfigs(
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
            .withKP(0)
            .withKI(0)
            .withKD(0)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);

        slot1Configs
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0)
            .withKI(0)
            .withKD(0);

        this.indexMotor.motor.getConfigurator().apply(slot1Configs);

        this.indexMotor.resetMotor();
    }

    private void setIndexSpeed(double rpm) {
        this.indexMotor.motor.setControl(
            new VelocityVoltage(rpm / 60)
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(false)
        );
    }

    public void startIndex() {
        this.setIndexSpeed(indexSpeed.getNumber());
    }

    public void reverseIndex() {
        this.setIndexSpeed(indexReverseSpeed.getNumber());
    }

    public void stopIndex() {
        this.setIndexSpeed(0);
    }

    public void khangaiIsAChud() {
        this.indexMotor.motor.setPosition(this.indexMotor.motor.getPosition().getValueAsDouble() % dyeRotorGearRatio);
        this.indexMotor.motor.setControl(new PositionVoltage(indexSafePosition.getNumber()).withSlot(1));
    }

    public boolean inSafePosition() {
        return Math.abs(this.indexMotor.motor.getPosition().getValueAsDouble() - this.indexSafePosition.getNumber())
            < this.indexSafePositionTolerance.getNumber();
    }

    public Command khangaiIsAChudCommand() {
        return Commands.runOnce(() -> this.khangaiIsAChud(), this);
    }

    public Command startIndexCommand() {
        return Commands.runOnce(() -> this.startIndex(), this);
    }

    public Command reverseIndexCommand() {
        return Commands.runOnce(() -> this.reverseIndex(), this);
    }

    public Command stopIndexCommand() {
        return Commands.runOnce(() -> this.stopIndex(), this);
    }

    @Override
    public void periodic() {
        this.indexMotor.update();
    }

    public static Index getInstance() {
        if (instance == null) {
            instance = new Index();
        }
        return instance;
    }
}
