package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Turret extends SubsystemBase{
    private static Turret instance = null;

    //TODO: Most clockwise is min, most counterclockwise is max, aka CCW+
    //TODO: 0 angle @ centered to robot pointing robot releative front
    public static final double kMaxTurretRotation = 9.73; //todo
    public static final Rotation2d kMaxTurretAngle = Rotation2d.fromDegrees(240); //todo
    public static final double kMinTurretRotation = 0; //todo
    public static final Rotation2d kMinTurretAngle = Rotation2d.fromDegrees(-110); //todo

    private RedRockTalon turretMotor = new RedRockTalon(0, "turretMotor", "*"); //todo

    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("turret/normalize-reset-speed", -0.05); //todo
    private double nonClampedTargetRevolution;

    private Turret() {
        super("turret");

        this.turretMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(1.5) //TODO
            .withKI(0) //TODO
            .withKD(0) //TODO
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(850)
            .withMotionMagicCruiseVelocity(220)
            .withMotionMagicJerk(10000000)
        )
        .withSpikeThreshold(10)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
    }

    public void setTurretPosition(Rotation2d angle) {
        this.turretMotor.setMotionMagicPosition(
            MathUtil.clamp(this.angleToRotation(angle), kMinTurretRotation + 0.2, kMaxTurretRotation - 0.2)

        );
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void reset() {
        this.turretMotor.setControl(new CoastOut());
        this.turretMotor.setMotionMagicPosition(0d);
        this.turretMotor.setControl(new DutyCycleOut(0));
    }

    private void setNormalizeSpeed() {
        this.turretMotor.setControl(
            new DutyCycleOut(this.normalizeSpeed.getNumber())
        );
    }

    private double angleToRotation(Rotation2d angle) {
        double a = MathUtil.inputModulus(angle.getDegrees(), -180, 180);
        double min = MathUtil.inputModulus(kMinTurretAngle.getDegrees(), -180, 180);
        double max = MathUtil.inputModulus(kMaxTurretAngle.getDegrees(), -180, 180) + 360;
        if (a < min) a += 360;
        return ((kMaxTurretRotation - kMinTurretRotation) / (max - min)) * (a - min) + kMinTurretRotation;
    }

    private void aimToTarget() {
        Rotation2d angle;  //todo vission
        this.setTurretPosition(new Rotation2d(/*angle.getRadians()*/));
    }

    public Command aimToTargetCommand(){
        return Commands.runOnce(() -> this.aimToTarget(), this);
    }

    @Override
    public void periodic() {
        this.turretMotor.update();
    }


    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }
}