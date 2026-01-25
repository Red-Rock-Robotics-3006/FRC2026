package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;


public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    public static double kMaxHoodAngle = 85,
                         kMinHoodAngle = 50;
    public static double kMaxHoodRotation = 1,
                         kMinHoodRotation = 0;

    private final RedRockTalon shooterMotor = new RedRockTalon(0, "shooterMotor", "*"); //TODO 
    private final RedRockTalon hoodMotor = new RedRockTalon(0, "hoodMotor", "*"); //TODO 

    private Slot0Configs shooterSlot0Configs = new Slot0Configs();
    private Slot0Configs hoodSlot0Configs = new Slot0Configs();

    private MotionMagicConfigs shooterMotionMagicConfigs = new MotionMagicConfigs();
    private MotionMagicConfigs hoodMotionMagicConfigs = new MotionMagicConfigs();

    private CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true);

    private SmartDashboardNumber shooterAccel = new SmartDashboardNumber("shooter/shooter-accel-motion-magic", 75);

    // private SmartDashboardNumber hoodMotionAccel = new SmartDashboardNumber("hood/hood-mm-accel", 40);
    // private SmartDashboardNumber hoodMotionVelo = new SmartDashboardNumber("hood/hood-mm-velo", 40);

    // private SmartDashboardNumber shooterKs = new SmartDashboardNumber("shooter/ks", 0);
    // private SmartDashboardNumber shooterKa = new SmartDashboardNumber("shooter/ka", 0);
    // private SmartDashboardNumber shooterKv = new SmartDashboardNumber("shooter/kv", 0.133); //to be tuned;
    // private SmartDashboardNumber shooterKp = new SmartDashboardNumber("shooter/kp", 0.4); //to be tuned;
    // private SmartDashboardNumber shooterKi = new SmartDashboardNumber("shooter/ki", 0);
    // private SmartDashboardNumber shooterKd = new SmartDashboardNumber("shooter/kd", 0);

    // private SmartDashboardNumber hoodKs = new SmartDashboardNumber("hood/ks", 0);
    // private SmartDashboardNumber hoodKa = new SmartDashboardNumber("hood/ka", 0);
    // private SmartDashboardNumber hoodKv = new SmartDashboardNumber("hood/kv", 0); //to be tuned;
    // private SmartDashboardNumber hoodKp = new SmartDashboardNumber("hood/kp", 7); //to be tuned;
    // private SmartDashboardNumber hoodKi = new SmartDashboardNumber("hood/ki", 0);
    // private SmartDashboardNumber hoodKd = new SmartDashboardNumber("hood/kd", 0);

    private SmartDashboardNumber reverseRPM = new SmartDashboardNumber("reverse shot rpm", -750);

    private SmartDashboardNumber spikeThreshold = new SmartDashboardNumber("hood/hood-spike-threshold", 10.5);
    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("hood/hood-normalize-speed", -0.05);

    private SmartDashboardNumber pidTolerance = new SmartDashboardNumber("hood/hood-pid-tolerance", 0.1);
    private SmartDashboardNumber positionTolerance = new SmartDashboardNumber("hood/hood-position-tolerance", 0.1);//gyatt good googly moogly

    private double nonClampedTargetRevolution;
    private double requestedRPM;

    //for smartdashboard only
    private double targetHoodAngle;
    private double targetRPM;
    private double targetPosition;


    private Shooter() {
        super("Shooter");

        this.hoodMotor.withMotorOutputConfigs(
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
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
                
        this.shooterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )
        .withSlot0Configs(
            new Slot0Configs()
            .withKA(0) //TODO
            .withKS(0) //TODO
            .withKV(0) //TODO
            .withKP(0.05) //TODO
            .withKI(0) //TODO
            .withKD(0)  //TODO
        )
        .withMotionMagicConfigs(
            new MotionMagicConfigs()

            .withMotionMagicAcceleration(1300)
            .withMotionMagicCruiseVelocity(100)
        )
        .withSpikeThreshold(28)
        .withCurrentLimitConfigs(
            new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(45)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);


        // this.shooterMotor.withMotorOutputConfigs(
        //     new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive)
        //         .withPeakForwardDutyCycle(1d)
        //         .withPeakReverseDutyCycle(-1d)
        //         .withNeutralMode(NeutralModeValue.Brake)
        // );

        // this.hoodMotor.withMotorOutputConfigs(
        //     new MotorOutputConfigs()
        //         .withInverted(InvertedValue.CounterClockwise_Positive)
        //         .withPeakForwardDutyCycle(1d)
        //         .withPeakReverseDutyCycle(-1d)
        //         .withNeutralMode(NeutralModeValue.Brake)
        // ).with;


        // this.shooterSlot0Configs = new Slot0Configs()
        //     .withKS(shooterKs.getNumber())
        //     .withKA(shooterKa.getNumber())
        //     .withKV(shooterKv.getNumber())
        //     .withKP(shooterKp.getNumber())
        //     .withKI(shooterKi.getNumber())
        //     .withKD(shooterKd.getNumber());

        // this.hoodSlot0Configs = new Slot0Configs()
        //     .withKS(hoodKs.getNumber())
        //     .withKA(hoodKa.getNumber())
        //     .withKV(hoodKv.getNumber())
        //     .withKP(hoodKp.getNumber())
        //     .withKI(hoodKi.getNumber())
        //     .withKD(hoodKd.getNumber());

        // this.shooterMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicAcceleration(shooterAccel.getNumber());

        // this.hoodMotionMagicConfigs = new MotionMagicConfigs()
        //     .withMotionMagicAcceleration(hoodMotionAccel.getNumber())
        //     .withMotionMagicCruiseVelocity(hoodMotionVelo.getNumber());

        // this.shooterMotor.withSlot0Configs(shooterSlot0Configs);
        // this.shooterMotor.withMotionMagicConfigs(shooterMotionMagicConfigs);
        // this.shooterMotor.withCurrentLimitConfigs(shooterCurrentLimitsConfigs);
        // this.hoodMotor.withSlot0Configs(hoodSlot0Configs);
        // this.hoodMotor.withMotionMagicConfigs(hoodMotionMagicConfigs);
        // this.hoodMotor.withCurrentLimitConfigs(hoodCurrentLimitsConfigs);
    }

    public void setHoodAngle(double angle) {
        this.hoodMotor.setMotionMagicPosition(MathUtil.clamp(this.angleToRotation(angle), kMinHoodRotation + 0.07, kMaxHoodRotation)

        );
        this.targetHoodAngle = angle;
        this.targetPosition = this.angleToRotation(angle);
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void setShooterRPM(double rpm) {
        this.shooterMotor.setMotionMagicPosition((rpm / 60d)

        );
        this.targetRPM = rpm;
    }

    public void setRequestedRPM() {
        this.setShooterRPM(this.requestedRPM);
    }

    public void setReverseRPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedRPM();
    }

    public void resetHood() {
        this.hoodMotor.setControl(new CoastOut());
        this.hoodMotor.setMotionMagicPosition(0d);
    }

    private void setNormalizeSpeed() {
        this.hoodMotor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    private double angleToRotation(double angle) {
        return ((kMaxHoodRotation - kMinHoodRotation) / (kMaxHoodAngle - kMinHoodAngle)) * (angle - kMinHoodAngle) + kMinHoodRotation;
    }

    @Override
    public void periodic() {
        this.hoodMotor.update();
        this.shooterMotor.update();

    //     if (shooterKs.hasChanged()
    //     || shooterKv.hasChanged()
    //     || shooterKp.hasChanged()
    //     || shooterKi.hasChanged()
    //     || shooterKd.hasChanged()
    //     || shooterKa.hasChanged()) {
    //         shooterSlot0Configs.kS = shooterKs.getNumber();
    //         shooterSlot0Configs.kV = shooterKv.getNumber();
    //         shooterSlot0Configs.kP = shooterKp.getNumber();
    //         shooterSlot0Configs.kI = shooterKi.getNumber();
    //         shooterSlot0Configs.kD = shooterKd.getNumber();
    //         shooterSlot0Configs.kA = shooterKa.getNumber();

    //         if (!Utils.isSimulation()) this.shooterMotor.withSlot0Configs(shooterSlot0Configs);
    //         System.out.println("applyied");
    //     }

        // if (shooterAccel.hasChanged()) {
        //     shooterMotionMagicConfigs.MotionMagicAcceleration = shooterAccel.getNumber();
        //     this.shooterMotor.withMotionMagicConfigs(shooterMotionMagicConfigs);
        // }

        // SmartDashboard.putNumber("shooter/shooter-acceleration", this.shooterMotor.getAcceleration().getValueAsDouble());
        // SmartDashboard.putNumber("shooter/shooter-velocity", this.shooterMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("shooter/shooter-rpm-target", this.targetRPM);

        // if (hoodKs.hasChanged()
        //     || hoodKv.hasChanged()
        //     || hoodKp.hasChanged()
        //     || hoodKi.hasChanged()
        //     || hoodKd.hasChanged()
        //     || hoodKa.hasChanged()) {
        //     hoodSlot0Configs.kS = hoodKs.getNumber();
        //     hoodSlot0Configs.kV = hoodKv.getNumber();
        //     hoodSlot0Configs.kP = hoodKp.getNumber();
        //     hoodSlot0Configs.kI = hoodKi.getNumber();
        //     hoodSlot0Configs.kD = hoodKd.getNumber();
        //     hoodSlot0Configs.kA = hoodKa.getNumber();

        //     if (!Utils.isSimulation()) this.hoodMotor.withSlot0Configs(hoodSlot0Configs);
        //     System.out.println("applyied");
        // }

        // if (hoodMotionAccel.hasChanged() || hoodMotionVelo.hasChanged()) {
        //     hoodMotionMagicConfigs.MotionMagicAcceleration = this.hoodMotionAccel.getNumber();
        //     hoodMotionMagicConfigs.MotionMagicCruiseVelocity = this.hoodMotionVelo.getNumber();
        //     this.hoodMotor.withMotionMagicConfigs(this.hoodMotionMagicConfigs);
        // }

        // SmartDashboard.putNumber("hood/hood-target-position", this.targetPosition);
        // SmartDashboard.putNumber("hood/hood-position", this.hoodMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("hood/hood-target-angle", this.targetHoodAngle);
        // SmartDashboard.putNumber("hood/hood-torque-current", this.hoodMotor.getTorqueCurrent().getValueAsDouble());
        // SmartDashboard.putBoolean("hood/hood-at-spike", this.inSpikeCurrent());
        // SmartDashboard.putBoolean("hood/hood-is-ready", this.isReady());


        // SmartDashboard.putNumber("shooter/shooter-request-rpm", this.requestedRPM);
    
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}