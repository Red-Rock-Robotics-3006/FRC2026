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

    public static double kMaxHoodAngle = 85, //todo
                         kMinHoodAngle = 50; //todo
    public static double kMaxHoodRotation = 1, //todo
                         kMinHoodRotation = 0; //todo

    private final RedRockTalon shooterMotor1 = new RedRockTalon(0, "shooterMotor1", "*"); //TODO 
    private final RedRockTalon shooterMotor2 = new RedRockTalon(0, "shooterMotor2", "*"); //TODO 
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
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
                
        this.shooterMotor1.withMotorOutputConfigs(
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
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);

        this.shooterMotor2.withMotorOutputConfigs(
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
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(true)
        ).withTuningEnabled(false);
        
        this.resetHood();
    }

    public void setHoodAngle(double angle) {
        this.hoodMotor.setMotionMagicPosition(MathUtil.clamp(this.angleToRotation(angle), kMinHoodRotation + 0.07, kMaxHoodRotation)
        );

        this.targetHoodAngle = angle;
        this.targetPosition = this.angleToRotation(angle);
        this.nonClampedTargetRevolution = this.angleToRotation(angle);
    }

    public void setShooter1RPM(double rpm) {
        this.shooterMotor1.setMotionMagicPosition((rpm / 60d)

        );
        this.targetRPM = rpm;
    }

    public void setRequestedShooter1RPM() {
        this.setShooter1RPM(this.requestedRPM);
    }

    public void setShooter2RPM(double rpm) {
        this.shooterMotor2.setMotionMagicPosition((rpm / 60d)

        );
        this.targetRPM = rpm;
    }

    public void setRequestedShooter2RPM() {
        this.setShooter2RPM(this.requestedRPM);
    }

    public void setReverseShooter1RPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedShooter1RPM();
    }

    public void setReverseShooter2RPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedShooter2RPM();
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
        this.shooterMotor1.update();
        this.shooterMotor2.update();

    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}