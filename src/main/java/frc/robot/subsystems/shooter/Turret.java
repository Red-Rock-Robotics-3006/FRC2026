package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.util.LerpingSmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Turret extends SubsystemBase{
    private static boolean kEnableTurretTuning = true;
    private static Turret instance = null;

    private double targetTurretPositionMotorRotations = 0;

    private SmartDashboardNumber turretTolerance = new SmartDashboardNumber("turret/tolerance", 2, kEnableTurretTuning && true);

    private RedRockTalon turretMotor = new RedRockTalon(44, "turret-motor", "*");

    private CANcoder ccoderA = new CANcoder(45, "*");
    private CANcoder ccoderB = new CANcoder(46, "*");

    private LerpingSmartDashboardNumber turretRestrictions
        = new LerpingSmartDashboardNumber(
            0, 0, 
            540, 20, 
            "turret/angle-degrees", "turret/motor-rotations", 
            kEnableTurretTuning && true);

    private Turret() {
        super();

        turretMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0Configs(
            new Slot0Configs()
        );

        this.resetTurret();
    }

    public void resetTurret() {
        turretMotor.motor.setPosition(
            turretRestrictions.getValue(
                this.chineseRemainderTheorem(ccoderA.getPosition().getValueAsDouble(), ccoderB.getPosition().getValueAsDouble()).getDegrees()
            )
        );
    }

    private Rotation2d chineseRemainderTheorem(double encoder1, double encoder2) {
        return Rotation2d.kZero;
    }

    /**
     * Sets the target angle of the turret.
     * 
     * @param angle Desired angle of turret. 0 is facing forward on the robot, CCW+
     */
    public void setTurretAngle(Rotation2d angle) {
        this.setTurretPosition(
            turretRestrictions.getValue(angle.getDegrees())
        );
    }

    /**
     * sets the turret position in terms of motor rotations
     * 
     * @param position raw desired motor rotation of the turret
     */
    private void setTurretPosition(double position) {
        this.turretMotor.setMotionMagicPosition(
            MathUtil.clamp(
                position, 
                turretRestrictions.getMinOutput(), 
                turretRestrictions.getMaxOutput())
        );
        this.targetTurretPositionMotorRotations = position;
    }

    public boolean atTurretAngle() {
        return Math.abs(turretMotor.motor.getPosition().getValueAsDouble() - this.targetTurretPositionMotorRotations)
            < turretRestrictions.convertOutputByRate(turretTolerance.getNumber());
    }

    @Override
    public void periodic() {
        turretMotor.update();
    }
    
    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }
}
