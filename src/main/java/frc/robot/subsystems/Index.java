package frc.robot.subsystems;


import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Index extends SubsystemBase{
    private static Index instance = null;

    private final TalonFX m_indexFeedMotor = new TalonFX(0); //TODO
    private final TalonFX m_indexConveyorMotor = new TalonFX(0); //TODO
    private final TalonFX m_indexCenterMotor = new TalonFX(0); //TODO

    private SmartDashboardNumber feedMotorSpeed = new SmartDashboardNumber("Index/feedMotorSpeed", .1); //TODO
    private SmartDashboardNumber conveyorMotorSpeed = new SmartDashboardNumber("Index/conveyorMotorSpeed", .1); //TODO
    private SmartDashboardNumber centerMotorSpeed = new SmartDashboardNumber("Index/centerMotorSpeed", .1); //TODO

    private Index() {
        super("Index");

        this.m_indexFeedMotor.getConfigurator().apply(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_indexConveyorMotor.getConfigurator().apply(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        );

        this.m_indexCenterMotor.getConfigurator().apply(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        );

    }
    public void startIndexFeed(){
        this.m_indexFeedMotor.setControl(new DutyCycleOut(this.feedMotorSpeed.getNumber()));
    }

    public void stopIndexFeed(){
        this.m_indexFeedMotor.setControl(new DutyCycleOut(0));
    }

    public void startIndexConveyor(){
        this.m_indexConveyorMotor.setControl(new DutyCycleOut(this.conveyorMotorSpeed.getNumber()));
    }

    public void stopIndexConveyor(){
        this.m_indexConveyorMotor.setControl(new DutyCycleOut(0));
    }

    public void startIndexCenter(){
        this.m_indexCenterMotor.setControl(new DutyCycleOut(this.centerMotorSpeed.getNumber()));
    }

    public void stopIndexCenter(){
        this.m_indexCenterMotor.setControl(new DutyCycleOut(0));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Index/feedMotorSpeed", feedMotorSpeed.getNumber());
        SmartDashboard.putNumber("Index/conveyorMotorSpeed", conveyorMotorSpeed.getNumber());
        SmartDashboard.putNumber("Index/centerMotorSpeed", centerMotorSpeed.getNumber());
    }

    public static Index getInstance(){
        if(instance == null)
            instance = new Index();
        return instance;
    }
}
