package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Index extends SubsystemBase{
    private static Index instance = null;

    private final RedRockTalon indexFeedMotor = new RedRockTalon(0, "indexFeedMotor", "*"); //TODO
    private final RedRockTalon indexConveyorMotor = new RedRockTalon(0, "indexConveyorMotor", "*"); //TODO
    private final RedRockTalon indexCenterMotor = new RedRockTalon(0, "indexCenterMotor", "*"); //TODO

    private SmartDashboardNumber feedMotorSpeed = new SmartDashboardNumber("Index/feedMotorSpeed", .1); //TODO
    private SmartDashboardNumber conveyorMotorSpeed = new SmartDashboardNumber("Index/conveyorMotorSpeed", .1); //TODO
    private SmartDashboardNumber centerMotorSpeed = new SmartDashboardNumber("Index/centerMotorSpeed", .1); //TODO

    private Index() {
        super("Index");

        this.indexFeedMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )

        this.indexConveyorMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )

        this.indexCenterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive) //todo make sure it is CCW
            .withPeakForwardDutyCycle(1d)
            .withPeakReverseDutyCycle(-1d)
            .withNeutralMode(NeutralModeValue.Brake)
        )


        public void startIndexFeed(){
            this.indexFeedMotor.setControl(feedMotorSpeed);
        }

        public void stopIndexFeed(){
            this.indexFeedMotor.setControl(0);
        }

        public void startIndexConveyor(){
            this.indexConveyorMotor.setControl(conveyorMotorSpeed);
        }

        public Void stopIndexConveyor(){
            this.indexConveyorMotor.setControl(0);
        }

        public void startIndexCenter(){
            this.indexCenterMotor.setControl(centerMotorSpeed);
        }

        public void stopIndexCenter(){
            this.indexCenterMotor.setControl(0);
        }
    }
}
