package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.autoaim.EditableShotParameter;
import frc.robot.subsystems.shooter.autoaim.ShotParameter;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.util.LerpingSmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private static final boolean kEnableShooterTuning = true;

    private double targetRPM = 0;
    private double targetHoodPositionMotorRotations = 0;

    private SmartDashboardNumber rpmTolerance = new SmartDashboardNumber("shooter/tolerance/rpm", 50, kEnableShooterTuning && true);
    private SmartDashboardNumber hoodTolerance = new SmartDashboardNumber("shooter/tolerance/hood", 2, kEnableShooterTuning && true);

    private RedRockTalon shooterLeftMotor = new RedRockTalon(41, "shooter-left-motor", "*");
    private RedRockTalon hoodMotor = new RedRockTalon(43, "shooter-hood-motor", "*");

    private EditableShotParameter manualShotParameter = new EditableShotParameter(30, 1000, "shooter/manual shot parameter");
    private EditableShotParameter lerpShotParameter = new EditableShotParameter(30, 200, "shooter/lerp shot parameter");
    
    private LerpingSmartDashboardNumber hoodRestrictions = 
        new LerpingSmartDashboardNumber(
            13, 0,
            50, 10.1,
            "shooter/hood/angle-degrees", "shooter/hood/motor-rotations", 
            kEnableShooterTuning && true);
    

    private Shooter() {
        shooterLeftMotor.withTuningEnabled(kEnableShooterTuning && true)
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0.25)
            .withKV(0.127)
            .withKP(0.45)
            .withKI(0)
            .withKD(0)
        ).withFollowerMotor(new TalonFX(42, "*"), MotorAlignmentValue.Opposed);

        hoodMotor.withTuningEnabled(kEnableShooterTuning && true)
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(3)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(1900)
                .withMotionMagicCruiseVelocity(50)
                .withMotionMagicJerk(9999)
        ).withResetSpeed(-0.2);

        hoodMotor.resetMotor();
    }

    /**
     * sets the angle of the hood, uses the utility lerping class to turn angle into raw motor pos to be set.
     * 
     * @param angle 0 is forward, 90 is up. angle of ELEVATION of the ball path, not the hood.
     */
    public void setHoodAngle(Rotation2d angle) {
        this.setHoodPosition(
            hoodRestrictions.getValue(angle.getDegrees())
        );
    }

    /**
     * sets the raw hood pos, but is clamped for min and max.
     * 
     * @param pos position of the hood in motor rotations
     */
    private void setHoodPosition(double pos) {
        hoodMotor.motor.setControl(
            new PositionVoltage(MathUtil.clamp(pos, hoodRestrictions.getMinOutput(), hoodRestrictions.getMaxOutput()))
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(true)
        );
        this.targetHoodPositionMotorRotations = pos;
    }

    public Command resetHoodCommand() {
        return hoodMotor.resetMotorCommand();
    }

    public void setShooterSpeed(double rpm) {
        if (Double.compare(0, rpm) == 0) {
            this.shooterLeftMotor.motor.setControl(
                new CoastOut()
            );
            return;
        }
        this.shooterLeftMotor.motor.setControl(
            new VelocityVoltage(rpm / 60)
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(false)
        );
        this.targetRPM = rpm;
    }

    /**
     * sets the angle of the hood AND the speed of the shooter according to the shot parameter
     */
    public void setShotParameter(ShotParameter shotParameter) {
        this.setHoodAngle(shotParameter.getHoodAngle());
        this.setShooterSpeed(shotParameter.rpm);
    }

    public boolean atShooterSpeed() {
        return Math.abs(shooterLeftMotor.motor.getVelocity().getValueAsDouble() * 60 - this.targetRPM) < this.rpmTolerance.getNumber();
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    public boolean atHoodAngle() {
        return Math.abs(hoodMotor.motor.getPosition().getValueAsDouble() - this.targetHoodPositionMotorRotations)
            < hoodRestrictions.convertOutputByRate(hoodTolerance.getNumber());
    }

    public void setManualShot() {
        this.setHoodAngle(manualShotParameter.getHoodAngle());
        this.setShooterSpeed(manualShotParameter.getShooterRPM());
    }

    public void setLerpTuneShot() {
        this.setHoodAngle(lerpShotParameter.getHoodAngle());
        this.setShooterSpeed(lerpShotParameter.getShooterRPM());
    }

    @Deprecated
    public boolean isReadyToShoot() {
        return this.atHoodAngle() && this.atShooterSpeed();
    }

    @Override
    public void periodic() {
        shooterLeftMotor.update();
        hoodMotor.update();
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}