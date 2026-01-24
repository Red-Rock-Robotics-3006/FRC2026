package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Localization;
import redrocklib.logging.SmartDashboardNumber;
import redrocklib.util.LerpingSmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Shooter extends SubsystemBase{
    private static Shooter instance = null;

    private static final boolean kEnableShooterTuning = true;

    public static final Pose2d shooterOffset = new Pose2d();

    private static final double halfFieldY = 10;

    private double targetRPM = 0;
    private double targetTurretPositionMotorRotations = 0;
    private double targetHoodPositionMotorRotations = 0;

    private SmartDashboardNumber rpmTolerance = new SmartDashboardNumber("shooter/tolerance/rpm", 50, kEnableShooterTuning && true);
    private SmartDashboardNumber hoodTolerance = new SmartDashboardNumber("shooter/tolerance/hood", 2, kEnableShooterTuning && true);
    private SmartDashboardNumber turretTolerance = new SmartDashboardNumber("turret/tolerance", 2, kEnableShooterTuning && true);

    private RedRockTalon shooterMotor = new RedRockTalon(0);
    private RedRockTalon hoodMotor = new RedRockTalon(0);
    private RedRockTalon turretMotor = new RedRockTalon(0);

    private CANcoder ccoderA = new CANcoder(29, "*"); //TODO
    private CANcoder ccoderB = new CANcoder(30, "*"); //TODO
    
    private LerpingSmartDashboardNumber hoodRestrictions = new LerpingSmartDashboardNumber(90, 0, 0, 1.2, 
                        "shooter/hood/Angle-Degrees", "shooter/hood/Motor Rotations", kEnableShooterTuning && true);
    
    private LerpingSmartDashboardNumber turretRestrictions
        = new LerpingSmartDashboardNumber(
            0, 0, 
            540, 20, 
            "turret/Angle-Degrees", "turret/motor-rotations", 
            kEnableShooterTuning && true);

    private enum ShooterState {
        RESTING,
        TURRET_TRACKING,
        AUTO_AIM,
        LOB,
        LOB_TRACKING,
        STOM //unused for now
    }

    private ShooterState state = ShooterState.RESTING;

    private Shooter() {
        shooterMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Coast)
        ).withSlot0Configs(
            new Slot0Configs()
        );

        hoodMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Coast)
        ).withSlot0Configs(
            new Slot0Configs()
        );

        turretMotor.withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Coast)
        ).withSlot0Configs(
            new Slot0Configs()
        );

        this.resetTurret();
    }

    /**
     * sets the angle of the hood, uses the utility lerping class to turn angle into raw motor pos to be set.
     * 
     * @param angle 0 is forward, 90 is up
     */
    public void setHoodAngle(Rotation2d angle) {
        this.setHoodPosition(
            hoodRestrictions.getValue(angle.getDegrees())
        );
    }

    /**
     * sets the raw hood pos, but is clamped for min and max.
     * 
     * @param pos position of the hood in motor rotations
     */
    private void setHoodPosition(double pos) {
        hoodMotor.motor.setControl(
            new PositionVoltage(MathUtil.clamp(pos, hoodRestrictions.getMinOutput(), hoodRestrictions.getMaxOutput()))
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(true)
        );
        this.targetHoodPositionMotorRotations = pos;
    }

    public Command resetHoodCommand() {
        Command m = hoodMotor.resetMotorCommand();
        m.addRequirements(this);
        return m;
    }

    public void resetTurret() {
        turretMotor.motor.setPosition(
            turretRestrictions.getValue(
                this.chineseRemainderTheorem(ccoderA.getPosition().getValueAsDouble(), ccoderB.getPosition().getValueAsDouble()).getDegrees()
            )
        );
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

    public void setShooterSpeed(double rpm) {
        this.shooterMotor.motor.setControl(
            new VelocityVoltage(rpm / 60)
            .withEnableFOC(true)
            .withSlot(0)
            .withOverrideBrakeDurNeutral(false)
        );
        this.targetRPM = rpm;
    }

    public void setShotParameter(ShotParameter shotParameter) {
        this.setHoodAngle(shotParameter.getHoodAngle());
        this.setShooterSpeed(shotParameter.rpm);
    }

    public boolean atShooterSpeed() {
        return Math.abs(shooterMotor.motor.getVelocity().getValueAsDouble() * 60 - this.targetRPM) < this.rpmTolerance.getNumber();
    }

    public boolean atHoodAngle() {
        return Math.abs(hoodMotor.motor.getPosition().getValueAsDouble() - this.targetHoodPositionMotorRotations)
            < hoodRestrictions.convertOutputByRate(hoodTolerance.getNumber());
    }

    public boolean atTurretAngle() {
        return Math.abs(turretMotor.motor.getPosition().getValueAsDouble() - this.targetTurretPositionMotorRotations)
            < turretRestrictions.convertOutputByRate(turretTolerance.getNumber());
    }

    public boolean isReadyToSturrett() {
        return this.atHoodAngle() && this.atShooterSpeed() && this.atTurretAngle();
    }

    public void setState(ShooterState state) {
        this.state = state;
    }

    public ShooterState getState() {
        return this.state;
    }

    @Override
    public void periodic() {

        Pose2d dtPose = CommandSwerveDrivetrain.getInstance().getPose();
        Pose2d turretPose = dtPose
                            .plus(
                                new Transform2d(
                                    new Pose2d(),
                                    shooterOffset.rotateBy(dtPose.getRotation())
                                )
                            );
        
        boolean isBlue = CommandSwerveDrivetrain.getInstance().isBlue();

        switch (state) {
            case RESTING:
                this.setHoodAngle(Rotation2d.kZero);
                this.setShooterSpeed(0);
                break;
            case LOB:
            case LOB_TRACKING:
                Pose2d[] lobPoses = isBlue ? Localization.blueLobTargets : Localization.redLobTargets;
                Pose2d targetPose = (dtPose.getY() > halfFieldY) ? lobPoses[1] : lobPoses[0];
                this.setTurretAngle(
                    turretPose
                        .relativeTo(targetPose)
                        .getTranslation()
                        .getAngle()
                        .minus(dtPose.getRotation())
                );

                if (this.state == ShooterState.LOB)
                    this.setShotParameter(
                        LobInterpolatingTable.get(
                            turretPose.minus(targetPose).getTranslation().getNorm()));
                else this.setShotParameter(new ShotParameter(Rotation2d.kZero, 0));
                break;
            case AUTO_AIM:
            case TURRET_TRACKING:
                Pose2d hubPose = isBlue ? Localization.blueHub : Localization.redHub;
                this.setTurretAngle(
                    turretPose
                        .relativeTo(hubPose)
                        .getTranslation()
                        .getAngle()
                        .minus(dtPose.getRotation())
                );

                if (this.state == ShooterState.AUTO_AIM)
                    this.setShotParameter(
                        LobInterpolatingTable.get(
                            turretPose.minus(hubPose).getTranslation().getNorm()));
                else this.setShotParameter(new ShotParameter(Rotation2d.kZero, 0));
                break;
        }

        shooterMotor.update();
        turretMotor.update();
        hoodMotor.update();
    }

    private Rotation2d chineseRemainderTheorem(double encoder1, double encoder2) {
        return Rotation2d.kZero;
    }

    public static Shooter getInstance() {
        if (instance == null) instance = new Shooter();
        return instance;
    }
}
