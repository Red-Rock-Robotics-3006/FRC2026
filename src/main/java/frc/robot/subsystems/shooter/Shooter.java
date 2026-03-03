package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private RedRockTalon shooterMotor = new RedRockTalon(41, "shooter-left-motor", "*");
    private RedRockTalon hoodMotor = new RedRockTalon(43, "shooter-hood-motor", "*");
    
    private LerpingSmartDashboardNumber hoodRestrictions = 
        new LerpingSmartDashboardNumber(
            10, 0, 
            45, 9, 
            "shooter/hood/angle-degrees", "shooter/hood/motor-rotations", 
            kEnableShooterTuning && true);
    

    private Shooter() {
        shooterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Coast)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0.133)
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
        ).withFollowerMotor(new TalonFX(42, "*"), MotorAlignmentValue.Opposed);

        hoodMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKA(0)
            .withKS(0)
            .withKV(0)
            .withKP(7)
            .withKI(0)
            .withKD(0)
            .withGravityType(GravityTypeValue.Elevator_Static)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(100)
                .withMotionMagicCruiseVelocity(400)
                .withMotionMagicJerk(10000)
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
        this.shooterMotor.motor.setControl(
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
        return Math.abs(shooterMotor.motor.getVelocity().getValueAsDouble() * 60 - this.targetRPM) < this.rpmTolerance.getNumber();
    }

    public double getTargetRPM() {
        return this.targetRPM;
    }

    public boolean atHoodAngle() {
        return Math.abs(hoodMotor.motor.getPosition().getValueAsDouble() - this.targetHoodPositionMotorRotations)
            < hoodRestrictions.convertOutputByRate(hoodTolerance.getNumber());
    }

    @Deprecated
    public boolean isReadyToShoot() {
        return this.atHoodAngle() && this.atShooterSpeed();
    }

    @Override
    public void periodic() {
        shooterMotor.update();
        hoodMotor.update();
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}