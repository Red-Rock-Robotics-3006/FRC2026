package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake speed", 0.2).withTuningEnabled(true);
    private SmartDashboardNumber intakeReverseSpeed = new SmartDashboardNumber("intake reverse speed", -0.2).withTuningEnabled(true);
    
    private RedRockTalon intakeMotor = new RedRockTalon(31, "intake", "*");

    private Intake() {
        this.intakeMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0.2)
            .withKI(0)
            .withKD(0)
        )
        .withSpikeThreshold(40)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
    }

    private void startIntaking() {
        this.intakeMotor.motor.set(intakeSpeed.getNumber());
    }

    private void stopIntaking() {
        this.intakeMotor.motor.set(0);
    }

    private void reverseIntake() {
        this.intakeMotor.motor.set(intakeReverseSpeed.getNumber());
    }

    public Command intakeCommand() {
        return Commands.runOnce(() -> startIntaking(), this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> stopIntaking(), this);
    }

    public Command reverseIntakeCommand() {
        return Commands.runOnce(() -> reverseIntake(), this);
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
