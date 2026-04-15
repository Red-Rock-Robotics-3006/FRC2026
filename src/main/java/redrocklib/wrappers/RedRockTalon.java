package redrocklib.wrappers;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import redrocklib.logging.SmartDashboardNumber;

public class RedRockTalon {
    public static final boolean kEnableMotorDashboardTuning = true;

    /**
     * main motor
     */
    public final TalonFX motor;

    private SmartDashboardNumber kS;
    private SmartDashboardNumber kA;
    private SmartDashboardNumber kV;
    private SmartDashboardNumber kP;
    private SmartDashboardNumber kI;
    private SmartDashboardNumber kD;
    private SmartDashboardNumber kG;

    private SmartDashboardNumber motionJerk;
    private SmartDashboardNumber motionAccel;
    private SmartDashboardNumber motionVel;

    private SmartDashboardNumber spikeThreshold;
    private SmartDashboardNumber resetSpeed;

    private boolean tuningEnabled = true;

    private Slot0Configs slot0Configs;
    private MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    private CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(50)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true);

    private MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    private FeedbackConfigs feedbackConfigs = null;

    private String name;

    private ArrayList<TalonFX> followerMotors = new ArrayList<>();

    public RedRockTalon(int motorID) {
        this(motorID, "motor-" + motorID);
    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param canivore
     */
    public RedRockTalon(int motorID, String name, String canivore) {
        this(motorID, name, canivore, new MotorOutputConfigs(), new Slot0Configs());
    }

    /**
     *Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     */
    public RedRockTalon(int motorID, String name) {
        this(motorID, name, new MotorOutputConfigs(), new Slot0Configs());
    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMEMBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param canivore
     * @param outPutConfigs
     * @param slot0Configs
     */
    public RedRockTalon(int motorID, String name, String canivore, MotorOutputConfigs outPutConfigs, Slot0Configs slot0Configs) {
        this.motor = new TalonFX(motorID, canivore);

        this.name = name;

        this.withMotorOutputConfigs(outPutConfigs)
            .withSlot0Configs(slot0Configs)
            .withCurrentLimitConfigs(this.currentLimitsConfigs)
            .withSpikeThreshold(5)
            .withMotionMagicConfigs(new MotionMagicConfigs())
            .withTuningEnabled(true);

        this.followerMotors.add(motor);
    }

    /**
     * Creates a wrapper for TalonFX that handles commonly used configs, commonly used telemetry, and updating config code
     * 
     * REMMEBER TO CALL {@link #update()} every loop somewhere in code
     * @param motorID
     * @param name
     * @param outPutConfigs
     * @param slot0Configs
     */
    public RedRockTalon(int motorID, String name, MotorOutputConfigs outPutConfigs, Slot0Configs slot0Configs) {
        this.motor = new TalonFX(motorID);

        this.name = name;

        this.withMotorOutputConfigs(outPutConfigs)
            .withSlot0Configs(slot0Configs)
            .withCurrentLimitConfigs(this.currentLimitsConfigs)
            .withSpikeThreshold(5)
            .withMotionMagicConfigs(new MotionMagicConfigs())
            .withTuningEnabled(true);
        
        this.followerMotors.add(motor);
    }

    public RedRockTalon withResetSpeed(double speed) {
        this.resetSpeed = new SmartDashboardNumber(name + "/" + name + "-reset-speed", speed, this.tuningEnabled);
        return this;
    }

    public RedRockTalon withTuningEnabled(boolean enabled) {
        this.tuningEnabled = kEnableMotorDashboardTuning && enabled;
        return this;
    }

    public RedRockTalon withSpikeThreshold(double threshold) {
        this.spikeThreshold = new SmartDashboardNumber(name + "/" + name + "-spike-threshold", threshold, this.tuningEnabled);
        return this;
    }

    public RedRockTalon withCurrentLimitConfigs(CurrentLimitsConfigs configs) {
        for (TalonFX follower : followerMotors)
            follower.getConfigurator().apply(configs);
        this.currentLimitsConfigs = configs;
        return this;
    }

    public RedRockTalon withMotorOutputConfigs(MotorOutputConfigs configs) {
        for (TalonFX follower : followerMotors)
            follower.getConfigurator().apply(configs);
        this.motorOutputConfigs = configs;
        return this;
    }

    public RedRockTalon withFeedbackConfigs(FeedbackConfigs configs){
        for (TalonFX follower : followerMotors)
            follower.getConfigurator().apply(configs);
        this.feedbackConfigs = configs;
        return this;
    }

    public RedRockTalon withSlot0Configs(Slot0Configs slot0Configs) {
        this.slot0Configs = slot0Configs;
        this.kS = new SmartDashboardNumber(name + "/" + name + "-slot0/kS", slot0Configs.kS, this.tuningEnabled);
        this.kA = new SmartDashboardNumber(name + "/" + name + "-slot0/kA", slot0Configs.kA, this.tuningEnabled);
        this.kV = new SmartDashboardNumber(name + "/" + name + "-slot0/kV", slot0Configs.kV, this.tuningEnabled);
        this.kP = new SmartDashboardNumber(name + "/" + name + "-slot0/kP", slot0Configs.kP, this.tuningEnabled);
        this.kI = new SmartDashboardNumber(name + "/" + name + "-slot0/kI", slot0Configs.kI, this.tuningEnabled);
        this.kD = new SmartDashboardNumber(name + "/" + name + "-slot0/kD", slot0Configs.kD, this.tuningEnabled);
        this.kG = new SmartDashboardNumber(name + "/" + name + "-slot0/kG", slot0Configs.kG, this.tuningEnabled);

        for (TalonFX follower : followerMotors)
            follower.getConfigurator().apply(slot0Configs);

        return this;
    }

    public RedRockTalon withMotionMagicConfigs(MotionMagicConfigs configs) {
        this.motionMagicConfigs = configs;
        this.motionAccel = new SmartDashboardNumber(name + "/" + name + "-motion-magic/accel", configs.MotionMagicAcceleration, this.tuningEnabled);
        this.motionVel = new SmartDashboardNumber(name + "/" + name + "-motion-magic/velocity", configs.MotionMagicCruiseVelocity, this.tuningEnabled);
        this.motionJerk = new SmartDashboardNumber(name + "/" + name + "-motion-magic/jerk", configs.MotionMagicJerk, this.tuningEnabled);

        for (TalonFX follower : followerMotors)
            follower.getConfigurator().apply(configs);

        return this;
    }

    public RedRockTalon withFollowerMotor(TalonFX follower, MotorAlignmentValue alignmentValue) {
        followerMotors.add(follower);
        for (TalonFX followerMotor : followerMotors) {
            followerMotor.getConfigurator().apply(this.motorOutputConfigs);
            followerMotor.getConfigurator().apply(this.slot0Configs);
            followerMotor.getConfigurator().apply(this.motionMagicConfigs);
            followerMotor.getConfigurator().apply(this.currentLimitsConfigs);
            if (this.feedbackConfigs != null)
                followerMotor.getConfigurator().apply(this.feedbackConfigs);
        }
        follower.setControl(new Follower(this.motor.getDeviceID(), alignmentValue));
        return this;
    }

    public boolean aboveSpikeThreshold() {
        return Math.abs(motor.getTorqueCurrent().getValueAsDouble()) >= spikeThreshold.getNumber();
    }

    public void setMotionMagicPosition(double position) {
        this.motor.setControl(
            new MotionMagicVoltage(position)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public void setMotionMagicVelocity(double rpm) {
        this.motor.setControl(
            new MotionMagicVelocityVoltage(rpm / 60d)
            .withSlot(0)
            .withEnableFOC(true)
            .withOverrideBrakeDurNeutral(true)
        );
    }

    public double getSpikeThreshold() {
        return this.spikeThreshold.getNumber();
    }

    public void setResetSpeed() {
        this.motor.setControl(new DutyCycleOut(resetSpeed.getNumber()));
    }

    public void resetMotor() {
        this.motor.setControl(new NeutralOut());
        for (TalonFX follower : this.followerMotors) follower.setPosition(0);
    }

    public Command resetMotorCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.setResetSpeed()),
            Commands.waitUntil(() -> this.aboveSpikeThreshold()),
            Commands.runOnce(() -> this.resetMotor())
        );
    }

    public Command resetMotorCommand(Subsystem requirement) {
        Command m = this.resetMotorCommand();
        m.addRequirements(requirement);
        return m;
    } 

    /**
     * Updates telemetry, and applies any changed config numbers for the motor
     * 
     * MUST BE CALLED EVERY LOOP
     */
    public void update() {
        if (tuningEnabled) {
            if (kS.hasChanged()
                || kA.hasChanged()
                || kV.hasChanged()
                || kP.hasChanged()
                || kI.hasChanged()
                || kD.hasChanged()
                || kG.hasChanged()) {
                slot0Configs.kS = kS.getNumber();
                slot0Configs.kA = kA.getNumber();
                slot0Configs.kV = kV.getNumber();
                slot0Configs.kP = kP.getNumber();
                slot0Configs.kI = kI.getNumber();
                slot0Configs.kD = kD.getNumber();
                slot0Configs.kG = kG.getNumber();
    
                for (TalonFX follower : followerMotors) {
                    follower.getConfigurator().apply(slot0Configs);
                }
            }
    
            if (motionVel.hasChanged() && Double.compare(motionVel.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicCruiseVelocity = motionVel.getNumber();
                for (TalonFX follower : followerMotors) 
                    follower.getConfigurator().apply(motionMagicConfigs);
            }
    
            if (motionAccel.hasChanged() && Double.compare(motionAccel.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicAcceleration = motionAccel.getNumber();
                for (TalonFX follower : followerMotors) 
                    follower.getConfigurator().apply(motionMagicConfigs);
            }
    
            if (motionJerk.hasChanged() && Double.compare(motionJerk.getNumber(), 0) != 0) {
                motionMagicConfigs.MotionMagicJerk = motionJerk.getNumber();
                for (TalonFX follower : followerMotors) 
                    follower.getConfigurator().apply(motionMagicConfigs);
            }
        }

        SmartDashboard.putNumber(name + "/" + name + "-current-position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-current-velocity-rps", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-current-velocity-rpm", motor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber(name + "/" + name + "-current-acceleration", motor.getAcceleration().getValueAsDouble());
        SmartDashboard.putNumber(name + "/" + name + "-torque-current", motor.getTorqueCurrent().getValueAsDouble());

        SmartDashboard.putBoolean(name + "/" + name + "-above-spike-threshold", this.aboveSpikeThreshold());
    }

}