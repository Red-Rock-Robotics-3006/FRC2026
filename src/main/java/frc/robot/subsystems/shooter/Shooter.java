package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;
import frc.robot.subsystems.Index;



public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    public static double kMaxHoodAngle = 85, //todo
                         kMinHoodAngle = 50; //todo
    public static double kMaxHoodRotation = 1, //todo
                         kMinHoodRotation = 0; //todo

    private final RedRockTalon shooterMotor1 = new RedRockTalon(0, "shooterMotor1", "*"); //TODO 
    private final RedRockTalon shooterMotor2 = new RedRockTalon(0, "shooterMotor2", "*"); //TODO 
    private final RedRockTalon hoodMotor = new RedRockTalon(0, "hoodMotor", "*"); //TODO 


    private SmartDashboardNumber reverseRPM = new SmartDashboardNumber("reverse shot rpm", -750);

    private SmartDashboardNumber normalizeSpeed = new SmartDashboardNumber("hood/hood-normalize-speed", -0.05);


    private double nonClampedTargetRevolution;
    private double requestedRPM;
    private double requestedHoodAngle;

    //for smartdashboard only
    private double targetHoodAngle;
    private double targetRPM;
    private double targetPosition;

    public boolean onBlue = true; //TODO: set this based on alliance color


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
        this.shooterMotor2.setMotionMagicPosition((-rpm / 60d)

        );
        this.targetRPM = rpm;
    }

    public void setRequestedShooter2RPM() {
        this.setShooter2RPM(-this.requestedRPM);
    }

    public void setReverseShooter1RPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedShooter1RPM();
    }

    public void setReverseShooter2RPM() {
        this.requestedRPM = reverseRPM.getNumber();
        this.setRequestedShooter2RPM();
    }

    public void setShooterRPM(double rpm) {
        this.setShooter1RPM(rpm);
        this.setShooter2RPM(rpm);
    }

    public void setRequestedShooterRPM() {
        this.setShooterRPM(this.requestedRPM);
    }

    public void resetHood() {
        this.hoodMotor.setControl(new CoastOut());
        this.hoodMotor.setMotionMagicPosition(0d);
    }

    private void setNormalizeSpeed() {
        this.hoodMotor.setControl(new DutyCycleOut(this.normalizeSpeed.getNumber()));
    }

    private void getTargetRed() {
        ShotParameter p = InterpolatingTable.getRed(3); //todo visuals
        this.requestedHoodAngle = p.pivotAngleDeg;
        this.requestedRPM = p.rpm;
    }

    private void getTargetBlue() {
        ShotParameter p = InterpolatingTable.getBlue(3); //todo visuals
        this.requestedHoodAngle = p.pivotAngleDeg;
        this.requestedRPM = p.rpm;
    }

    private double angleToRotation(double angle) {
        return ((kMaxHoodRotation - kMinHoodRotation) / (kMaxHoodAngle - kMinHoodAngle)) * (angle - kMinHoodAngle) + kMinHoodRotation;
    }

    public Command getTargetRedCommand() {
        return Commands.runOnce(() -> this.getTargetRed(), this);
    }

    public Command getTargetBlueCommand() {
        return Commands.runOnce(() -> this.getTargetBlue(), this);
    }

    public Command startShooterBlueCommand(){
        return Commands.sequence(
            this.getTargetBlueCommand(),
            Commands.runOnce(() -> this.setHoodAngle(requestedHoodAngle), this),
            Commands.runOnce(() -> this.setRequestedShooterRPM(), this),
            Commands.waitSeconds(1.5),
            Index.getInstance().startIndexCommand()
        );
    }

    public Command startShooterRedCommand(){
        return Commands.sequence(
            this.getTargetRedCommand(),
            Commands.runOnce(() -> this.setHoodAngle(requestedHoodAngle), this),
            Commands.runOnce(() -> this.setRequestedShooterRPM(), this),
            Commands.waitSeconds(1.5),
            Index.getInstance().startIndexCommand()
        );
    }

    public Command stopShooterCommand(){
        return Commands.sequence(
            Index.getInstance().stopIndexCommand(),
            Commands.runOnce(() -> this.setShooterRPM(0), this)
        );
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