package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private RedRockTalon driveMotor = new RedRockTalon(21, "intake-drive-motor", "*");
    private RedRockTalon extensionMotor = new RedRockTalon(22, "intake-extension-motor", "*");
    
    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/drive/intake speed", 0.8333);
    private SmartDashboardNumber intakeReverseSpeed = new SmartDashboardNumber("intake/drive/intake reverse speed", -0.6);

    private SmartDashboardNumber maxPivotRotation = new SmartDashboardNumber("intake/extension/max rotation", 10);
    private SmartDashboardNumber minPivotRotation = new SmartDashboardNumber("intake/extension/min rotation", 0);

    private SmartDashboardNumber intakeDeployPosition = new SmartDashboardNumber("intake/extension/deploy position", 10);
    private SmartDashboardNumber intakeStowPosition = new SmartDashboardNumber("intake/extension/stow position", 0);
    private SmartDashboardNumber intakePushRetractPosition = new SmartDashboardNumber("intake/extension/push retract position", 4);
    private SmartDashboardNumber intakePushDeployPosition = new SmartDashboardNumber("intake/extension/push deploy position", 6);
    private SmartDashboardNumber intakePositionTolerance = new SmartDashboardNumber("intake/extension/position tolerance", 0.1);

    private double targetPosition = 0;
    
    private Intake() {
        super();
                
        this.driveMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
        
        this.extensionMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
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
            .withGravityType(GravityTypeValue.Arm_Cosine)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(220)
            .withMotionMagicJerk(10000000)
        ).withSpikeThreshold(10)
        .withResetSpeed(0.1)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);

        this.extensionMotor.resetMotor();
    }

    public void setExtensionPosition(double rotations) {
        this.extensionMotor.setMotionMagicPosition(MathUtil.clamp(rotations, minPivotRotation.getNumber(), maxPivotRotation.getNumber()));
        this.targetPosition = rotations;
    }

    public boolean atTargetPosition() {
        return Math.abs(extensionMotor.motor.getPosition().getValueAsDouble() - this.targetPosition)
            < this.intakePositionTolerance.getNumber();
    }

    public void setDriveSpeed(double speed) {
        this.driveMotor.motor.setControl(new DutyCycleOut(speed));
    }

    public void deployIntake() {
        this.setExtensionPosition(intakeDeployPosition.getNumber());
    }

    public void stowIntake() {
        this.setExtensionPosition(intakeStowPosition.getNumber());
    }

    public void pushRetractIntake() {
        this.setExtensionPosition(intakePushRetractPosition.getNumber());
    }

    public void pushDeployIntake() {
        this.setExtensionPosition(intakePushDeployPosition.getNumber());
    }

    public void startIntake() {
        this.setDriveSpeed(intakeSpeed.getNumber());
    }

    public void reverseIntake() {
        this.setDriveSpeed(intakeReverseSpeed.getNumber());
    }

    public void stopIntake() {
        this.setDriveSpeed(0);
    }

    public Command startIntakeCommand() {
        return Commands.sequence( 
            Commands.runOnce(() -> this.deployIntake(), this),
            Commands.runOnce(() -> this.startIntake(), this),
            Commands.waitUntil(() -> this.atTargetPosition())
        );
    }

    public Command reverseIntakeCommand() {
        return Commands.runOnce(() -> this.reverseIntake(), this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> this.stopIntake(), this);
    }

    public Command stowIntakeCommand() {
        return Commands.runOnce(() -> this.stowIntake(), this);
    }

    public Command deployIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.deployIntake(), this),
            extensionMotor.resetMotorCommand()
        );
    }

    public Command pulsateIntakeCommand() {
        return Commands.sequence(
            Commands.waitSeconds(2),
            Commands.sequence(
                Commands.runOnce(() -> this.pushRetractIntake(), this),
                Commands.waitUntil(() -> this.atTargetPosition()),
                Commands.waitSeconds(0.75),
                Commands.runOnce(() -> this.pushDeployIntake(), this),
                Commands.waitUntil(() -> this.atTargetPosition())
            ).repeatedly()
        );
    }

    public Command resetPivotCommand() {
        return this.extensionMotor.resetMotorCommand();
    }

    @Override
    public void periodic() {
        this.driveMotor.update();
        this.extensionMotor.update();
    }

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }
}