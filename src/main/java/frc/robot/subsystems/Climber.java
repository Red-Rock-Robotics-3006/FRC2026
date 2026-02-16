package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Climber extends SubsystemBase{
    private static Climber instance = null;

    private final RedRockTalon climberMotor = new RedRockTalon(0, "climberMotor", "*"); //TODO 


    private SmartDashboardNumber climb1Position = new SmartDashboardNumber("Climber/climb1Position", 28.6); //TODO
    private SmartDashboardNumber climb2Position = new SmartDashboardNumber("Climber/climb2Position", 28.6); //TODO
    private SmartDashboardNumber climb3Position = new SmartDashboardNumber("Climber/climb3Position", 28.6); //TODO
    private SmartDashboardNumber stowPosition = new SmartDashboardNumber("Climber/stowPosition", 3); //TODO

    private Climber() {
        super("Climber");
                
        this.climberMotor.withMotorOutputConfigs(
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

        this.resetClimber();
    }


    public void setClimb1Position(){
        this.climberMotor.setMotionMagicPosition(this.climb1Position.getNumber());
    }

    public void setClimb2Position(){
        this.climberMotor.setMotionMagicPosition(this.climb2Position.getNumber());
    }

    public void setClimb3Position(){
        this.climberMotor.setMotionMagicPosition(this.climb3Position.getNumber());
    }

    public void resetClimber(){
        this.climberMotor.motor.setPosition(0); //TODO
    }

    public void setClimbStowPosition(){
        this.climberMotor.setMotionMagicPosition(this.stowPosition.getNumber());
    }

    public Command resetClimberCommand(){
        return Commands.sequence(
            Commands.waitUntil(() ->  this.climberMotor.aboveSpikeThreshold()),
            Commands.runOnce(() -> this.resetClimber(), this)
        );
    }

    public Command setCLimb1Command(){
        return Commands.runOnce(() -> this.setClimb1Position(), this);
    }

    public Command setClimb2Command(){
        return Commands.runOnce(() -> this.setClimb2Position(), this);
    }

    public Command setClimb3Command(){
        return Commands.runOnce(() -> this.setClimb3Position(), this);
    }

    public Command stowClimberCommand(){
        return Commands.runOnce(() -> this.setClimbStowPosition(), this);
    }

    @Override
    public void periodic(){
        this.climberMotor.update();
    }

    public static Climber getInstance(){
        if(instance == null)
            instance = new Climber();
        return instance;
    }
}
