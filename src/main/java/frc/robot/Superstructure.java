package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.vision.Localization;
// import frc.robot.vision.Localization;
import redrocklib.logging.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    public static Localization localizationInstance = new Localization();

    public static final boolean kPractice = true;
    public static final boolean kHubOrLob = true; //true for hub, false for lob

    // private final LED led = LED.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Index index = Index.getInstance();
    // private final Climber climber = Climber.getInstance();

    private SmartDashboardNumber blueAllianceZoneX = new SmartDashboardNumber("superstructure/blue alliance zone x", 4.38);
    private SmartDashboardNumber redAllianceZoneX = new SmartDashboardNumber("superstructure/red alliance zone x", 12.16);

    public enum RobotState {
        TURRET_TRACKING, //turret tracking
        TRACKING, //flywheels spin up, hood tracking, turret tracking
        SHOOTING, //index spinning, flywheels spinning, hood tracking, turret tracking
        SHOOTING_WHILE_MOVING, //flywheels spinning, hood tracking, turret tracking, dt speed > 0

        HUB_SHOT, //shooting from in front of hub, flywheels spinning, turret at set angle, hood at set angle

        CLIMBING, //stow mechs, might need substates later
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

    public static final Pose2d shooterOffset = new Pose2d();

    @Override
    public void periodic() {
        dtPose = drivetrain.getPose();
        shooterPose = dtPose.plus(new Transform2d(new Pose2d(), shooterOffset.rotateBy(dtPose.getRotation()))); //this one is so clean and good
        SmartDashboard.putBoolean("superstructure/in alliance zone", inAllianceZone());
        boolean isBlue = drivetrain.isBlue();

        // this.targetPose = this.inAllianceZone() ? 
        //     ((isBlue) ? Localization.blueHub : Localization.redHub) :
        //     new Pose2d();//will be lob poses later

        switch (robotState) {
            case HUB_SHOT:
                //set shooter, turret targets to fixed values for hub shot -> not needed
                if (readyToShoot())
                    index.startFeed();
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
                // led.setLights(led.GREEN);
            case TRACKING:
                // if (readyToShoot()) 
                //     setState(RobotState.SHOOTING);

                // shooter.setShotParameter(
                //     (this.inAllianceZone()) ? 
                //         InterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm()) :
                //         LobInterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm())
                // );
                
                // led.blink(led.GREEN, 4);
            case TURRET_TRACKING:
                // turret.setTurretAngle(
                //     shooterPose
                //         .relativeTo(targetPose)
                //         .getTranslation()
                //         .getAngle()
                //         .minus(dtPose.getRotation())
                // );
                
                // led.rainbow();
                break;

            case CLIMBING:
                //retract intake
                //then climb logic
                break;
            case IDLE:
                // led.rainbow();
                break;
        }
    }

    public void setManualShotParameter(ShotParameter shot) {
        this.shooter.setShotParameter(shot);
        this.setState(RobotState.HUB_SHOT);
    }

    public Command setManualShotParameterCommand(ShotParameter shot) {
        return Commands.runOnce(() -> setManualShotParameter(shot), this);
    }

    public Command setManualIndexStartCommand() {
        return index.startFeedCommand();
    }

    public Command setManualIndexStopCommand() {
        return index.stopFeedCommand();
    }

    public Command resetShooterHoodCommand() {
        return shooter.resetHoodCommand();
    }

    public void setState(RobotState state) {
        switch (state) {
            case IDLE:
                shooter.setShooterSpeed(0);
                shooter.setHoodAngle(Rotation2d.kZero);
                break;
            case SHOOTING:
                index.startFeed();
            case TURRET_TRACKING:
                index.stopFeed();
                shooter.setShotParameter(new ShotParameter(10, 0));
                break;
            default:
                break;
        }
        this.robotState = state;
    }
    
    public Command setStateCommand(RobotState state) {
        return Commands.runOnce(() -> setState(state), this);
    }

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }
}