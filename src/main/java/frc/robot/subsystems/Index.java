package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

    private RedRockTalon indexMotor = new RedRockTalon(31, "index", "*");

    private SmartDashboardNumber indexSpeed = new SmartDashboardNumber("index/index speed", 4500).withTuningEnabled(true);
    private SmartDashboardNumber indexReverseSpeed = new SmartDashboardNumber("index/index reverse speed", -3000).withTuningEnabled(true);

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
