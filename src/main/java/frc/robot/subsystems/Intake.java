package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    private RedRockTalon driveMotor = new RedRockTalon(21, "intake-drive", "*");
    private RedRockTalon pivotMotor = new RedRockTalon(22, "intake-pivot", "*");

    private SmartDashboardNumber maxPivotRotation = new SmartDashboardNumber("intake/pivot/max rotation", 0); 
    private SmartDashboardNumber minPivotRotation = new SmartDashboardNumber("intake/pivot/min rotation", 0); 
    private SmartDashboardNumber intakeDeployPosition = new SmartDashboardNumber("intake/pivot/deploy position", 10); 
    private SmartDashboardNumber intakeStowPosition = new SmartDashboardNumber("intake/pivot/stow position", 0); 

    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("intake/drive/intake speed RPM", 1000);
    private SmartDashboardNumber intakeReverseSpeed = new SmartDashboardNumber("intake/drive/intake reverse speed RPM", -400);

    private Intake() {
        super();
        
        this.pivotMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(0.5)
            .withKI(0)
            .withKD(0) //TODO tune
            .withGravityType(GravityTypeValue.Arm_Cosine)
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(220)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(10) //TODO tune
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(true);
                
        this.driveMotor.withMotorOutputConfigs(
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

        this.pivotMotor.resetMotor();
    }

    public void setPivotPosition(double rotations) {
        this.pivotMotor.setMotionMagicPosition(MathUtil.clamp(rotations, minPivotRotation.getNumber(), maxPivotRotation.getNumber()));
    }

    public void setDriveSpeed(double speed) {
        this.driveMotor.motor.setControl(
            new VelocityVoltage(speed / 60)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public void deployIntake() {
        this.setPivotPosition(intakeDeployPosition.getNumber());
    }

    public void stowIntake() {
        this.setPivotPosition(intakeStowPosition.getNumber());
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

    public Command reverseIntakeCommand() {
        return Commands.runOnce(() -> this.reverseIntake(), this);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> this.stopIntake(), this);
    }

    public Command startIntakeCommand() {
        return Commands.sequence( 
                Commands.runOnce(() -> this.deployIntake(), this),
                Commands.runOnce(() -> this.startIntake(), this)
            );
    }

    public Command stowIntakeCommand() {
        return Commands.runOnce(() -> this.stowIntake(), this);
    }

    public Command resetPivotCommand() {
        return this.pivotMotor.resetMotorCommand();
    }

    @Override
    public void periodic() {
        this.driveMotor.update();
        this.pivotMotor.update();
    }

    public static Intake getInstance() {
        if (instance == null) instance = new Intake();
        return instance;
    }
}