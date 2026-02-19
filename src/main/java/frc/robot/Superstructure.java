package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.autoaim.InterpolatingTable;
import frc.robot.subsystems.shooter.autoaim.LobInterpolatingTable;
import frc.robot.subsystems.shooter.autoaim.SOTMCalcs;
import frc.robot.subsystems.shooter.autoaim.ShotParameter;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.Localization;
import redrocklib.logging.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    public static final boolean kPractice = true;
    public static final boolean kHubOrLob = true; //true for hub, false for lob

    private final Shooter shooter = Shooter.getInstance();
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Index index = Index.getInstance();
    public final Localization localization = Localization.getInstance();

    private SmartDashboardNumber blueAllianceZoneX = new SmartDashboardNumber("superstructure/blue alliance zone x", 4.38);
    private SmartDashboardNumber redAllianceZoneX = new SmartDashboardNumber("superstructure/red alliance zone x", 12.16);

    private final ShotParameter IDLE = new ShotParameter(10, 0);

    public enum RobotState {
        TURRET_TRACKING, //turret tracking
        FULL_TRACKING, //flywheels spin up, hood tracking, turret tracking
        SHOOTING, //index spinning, flywheels spinning, hood tracking, turret tracking
        SHOOTING_WHILE_MOVING, //mostly for redundancy, flywheels spinning, hood tracking, turret tracking, dt speed > 0

        MANUAL_SHOT, //any manual shot, flywheels spinning, turret at set angle, hood at set angle

        IDLE //mechanisms all idle (on disable maybe?)
    }

    private RobotState robotState = RobotState.IDLE;

    private boolean inAllianceZone() {
        if (kPractice) return kHubOrLob;
        return drivetrain.isBlue() ? shooterPose.getX() < blueAllianceZoneX.getNumber() : shooterPose.getX() > redAllianceZoneX.getNumber();
    }

    private boolean readyToShoot() {
        return shooter.atHoodAngle() && shooter.atShooterSpeed() && turret.atTurretAngle();
    }

    private Pose2d dtPose;
    private Pose2d shooterPose;
    private Pose2d targetPose = new Pose2d();

    public static final Pose2d shooterOffset = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    @Override
    public void periodic() {
        SwerveDriveState state = drivetrain.getState();
        dtPose = state.Pose;
        shooterPose = dtPose.plus(new Transform2d(new Pose2d(), shooterOffset.rotateBy(dtPose.getRotation()))); //this one is so clean and good
        SmartDashboard.putBoolean("superstructure/in alliance zone", inAllianceZone());
        boolean isBlue = drivetrain.isBlue();

        this.targetPose = (this.inAllianceZone() ? 
            ((isBlue) ? Localization.blueHub : Localization.redHub) :
            new Pose2d()) //will be lob poses later
            .transformBy(new Transform2d(SOTMCalcs.getOffset(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond), new Rotation2d()));

        switch (robotState) {
            case MANUAL_SHOT:
                if (readyToShoot()) index.startIndex();
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
            case FULL_TRACKING:
                if (readyToShoot()) setState(RobotState.SHOOTING);

                shooter.setShotParameter(
                    (this.inAllianceZone()) ? 
                        InterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm()) :
                        LobInterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm())
                );
            case TURRET_TRACKING:
                turret.setTurretAngle(
                    shooterPose
                        .relativeTo(targetPose)
                        .getTranslation()
                        .getAngle()
                        .minus(dtPose.getRotation())
                );
                break;
            case IDLE:
                break;
        }
    }

    public void setState(RobotState state) {
        switch (state) {
            case IDLE:
                index.stopIndex();
                shooter.setShotParameter(IDLE);
                break;
            case SHOOTING:
                index.startIndex();
                break;
            case TURRET_TRACKING:
                index.stopIndex();
                shooter.setShotParameter(IDLE);
                break;
            default:
                break;
        }
        this.robotState = state;
    }
    
    public Command setStateCommand(RobotState state) {
        return Commands.runOnce(() -> setState(state), this);
    }

    public RobotState getRobotState() {
        return this.robotState;
    }

    public void setManualShotParameter(ShotParameter shot) {
        this.shooter.setShotParameter(shot);
        this.setState(RobotState.MANUAL_SHOT);
    }

    public Command setManualShotParameterCommand(ShotParameter shot) {
        return Commands.runOnce(() -> setManualShotParameter(shot), this);
    }

    public Command resetShooterHoodCommand() {
        return shooter.resetHoodCommand();
    }

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }
}
