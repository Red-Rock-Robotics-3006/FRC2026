package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    private SmartDashboardNumber intakeSpeedDCO = new SmartDashboardNumber("intake speed duty cycle", 0.15).withTuningEnabled(true);
    private SmartDashboardNumber intakeReverseSpeedDCO = new SmartDashboardNumber("intake reverse speed duty cycle", -0.15).withTuningEnabled(true);
    private SmartDashboardNumber intakeSpeedRPM = new SmartDashboardNumber("intake speed rpm", 80).withTuningEnabled(true);
    private SmartDashboardNumber intakeReverseSpeedRPM = new SmartDashboardNumber("intake reverse speed rpm", -80).withTuningEnabled(true);

    private RedRockTalon intakeMotor = new RedRockTalon(31, "intake");

    private Intake() {
        this.intakeMotor
        .withTuningEnabled(true)
        .withMotorOutputConfigs(
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
            .withKP(0)
            .withKI(0)
            .withKD(0)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(100)
            .withMotionMagicAcceleration(850)
            .withMotionMagicJerk(10000000)
        )
        .withFollowerMotor(new TalonFX(32), MotorAlignmentValue.Aligned);
    }

    private void startIntaking() {
        this.intakeMotor.motor.setControl(new DutyCycleOut(intakeSpeedDCO.getNumber()));
    }

    private void stopIntaking() {
        this.intakeMotor.motor.setControl(new DutyCycleOut(0));
    }

    private void reverseIntake() {
        this.intakeMotor.motor.setControl(new DutyCycleOut(intakeReverseSpeedDCO.getNumber()));
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

    private void startIntakingRPM() {
        this.intakeMotor.setMotionMagicVelocity(this.intakeSpeedRPM.getNumber());
    }

    private void reverseIntakeRPM() {
        this.intakeMotor.setMotionMagicVelocity(this.intakeReverseSpeedRPM.getNumber());
    }

    public Command intakeRPMCommand() {
        return Commands.runOnce(() -> startIntakingRPM(), this);
    }

    public Command reverseIntakeRPMCommand() {
        return Commands.runOnce(() -> reverseIntakeRPM(), this);
    }

    @Override
    public void periodic() {
        this.intakeMotor.update();
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }
}
