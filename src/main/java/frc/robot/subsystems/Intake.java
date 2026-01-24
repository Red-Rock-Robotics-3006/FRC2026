package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Intake extends SubsystemBase{
    private static Intake instance = null;

    private final RedRockTalon intakeMotor = new RedRockTalon(0, "intakeMotor", "*"); //TODO 
    private final RedRockTalon pivotMotor = new RedRockTalon(0, "Motor", "*"); //TODO

    private SmartDashboardNumber deployPosition = new SmartDashboardNumber("Intake/deployPosition", 28.6); //TODO
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("Intake/stowPosition", 3); //TODO
    private SmartDashboardNumber intakeSpeed = new SmartDashboardNumber("Intake/intakeSpeed", 0.4); //TODO
    private SmartDashboardNumber outtakeSpeed = new SmartDashboardNumber("Intake/outtakeSpeed", -0.4); //TODO

    private Intake() {
        super("Intake");
                
        this.pivotMotor.withMotorOutputConfigs(
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
                
        this.intakeMotor.withMotorOutputConfigs(
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

        this.resetIntake();
    }

    public void setOuttakeSpeedRPM(){ //Used if fuel gets stuck in intake
        this.intakeMotor.motor.set(this.outtakeSpeed.getNumber());
    }    

    public void setIntakeSpeedRPM(){
        this.intakeMotor.motor.set(this.intakeSpeed.getNumber());
    }

    public void stopIntake(){
        this.intakeMotor.motor.set(0);
    }

    public void setIntakePosition(){
        this.pivotMotor.setMotionMagicPosition(this.deployPosition.getNumber());
    }

    public void normalizeIntake(){
        this.pivotMotor.motor.set(-0.1); //TODO
    }

    public void resetIntake(){
        this.pivotMotor.motor.setControl(new NeutralOut());
        this.pivotMotor.motor.setPosition(0); //TODO
    }
    public void setIntakeStowPosition(){
        this.pivotMotor.setMotionMagicPosition(this.stowPosition.getNumber());
    }

    public boolean isIntaking(){
        return this.pivotMotor.motor.getPosition().getValueAsDouble() > 5;
    }

    public Command startIntakeCommand(){
        return Commands.runOnce(() -> this.setIntakeSpeedRPM(), this);
    }
    
    public Command stopIntakeCommand(){
        return Commands.runOnce(() -> this.stopIntake(), this);
    }

    public Command startOttakeCommand(){ //Used if fuel gets stuck in intake
        return Commands.runOnce(() -> this.setIntakeSpeedRPM(), this);
    }

    public Command resetIntakeCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> this.normalizeIntake(), this),
            Commands.waitUntil(() ->  this.pivotMotor.aboveSpikeThreshold()),
            Commands.runOnce(() -> this.resetIntake(), this)
        );
    }

    public Command deployIntakeCommand(){
        return Commands.runOnce(() -> this.setIntakePosition(), this);
    }

    public Command stowIntakeCommand(){
        return Commands.runOnce(() -> this.setIntakeStowPosition(), this);
    }

    @Override
    public void periodic(){
        this.intakeMotor.update();
        this.pivotMotor.update();
    }

    public static Intake getInstance(){
        if(instance == null)
            instance = new Intake();
        return instance;
    }
}