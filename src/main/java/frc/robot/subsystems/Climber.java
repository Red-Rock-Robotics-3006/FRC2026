package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Climber extends SubsystemBase{
    private static Climber instance = null;

    private RedRockTalon climbMotor = new RedRockTalon(51, "climber", "*");

    private SmartDashboardNumber raiseSpeed = new SmartDashboardNumber("climber/raise-speed", 0.9);
    private SmartDashboardNumber lowerSpeed = new SmartDashboardNumber("climber/lower-speed", -0.9);

    private Climber() {
        super("climber");

        this.climbMotor
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withTuningEnabled(false);
    }

    public void raiseClimber() {
        this.climbMotor.motor.setControl(new DutyCycleOut(raiseSpeed.getNumber()));
    }
    
    public void lowerClimber() {
        this.climbMotor.motor.setControl(new DutyCycleOut(lowerSpeed.getNumber()));
    }
    
    public void stopClimber() {
        this.climbMotor.motor.setControl(new NeutralOut());
    }

    public Command raiseClimberCommand() {
        return Commands.runOnce(() -> raiseClimber(), this);
    }

    public Command lowerClimberCommand() {
        return Commands.runOnce(() -> lowerClimber(), this);
    }

    public Command stopClimberCommand() {
        return Commands.runOnce(() -> stopClimber(), this);
    }

    @Override
    public void periodic() {
        climbMotor.update();
    }

    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }
}