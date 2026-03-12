package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.autoaim.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.vision.Localization;

import redrocklib.logging.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    public static final boolean kPractice = false;
    public static final boolean kHubOrLob = true; //true for hub, false for lob

    public final Intake intake = Intake.getInstance();
    private final Index index = Index.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Turret turret = Turret.getInstance();
    private final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public final Localization localization = Localization.getInstance();

    private SmartDashboardNumber blueAllianceZoneX = new SmartDashboardNumber("superstructure/blue alliance zone x", 4.38);
    private SmartDashboardNumber redAllianceZoneX = new SmartDashboardNumber("superstructure/red alliance zone x", 12.16);

    private final ShotParameter IDLE = new ShotParameter(10, 0);

    private Field2d field2d = new Field2d();
    private FieldObject2d fieldObject2d;

    public enum RobotState {
        TURRET_TRACKING, //turret tracking
        FULL_TRACKING, //flywheels spin up, hood tracking, turret tracking
        SHOOTING, //index spinning, flywheels spinning, hood tracking, turret tracking
        SHOOTING_WHILE_MOVING, //mostly for redundancy, flywheels spinning, hood tracking, turret tracking, dt speed > 0

        MANUAL_SHOT, //any manual shot, flywheels spinning, turret at set angle, hood at set angle

        LERP_TUNING, //for tuning the lerp table (prob dont need this state for comp), turret tracking, hood at set angle, flywheels at set rpm

        IDLE //mechanisms all idle (on disable maybe?)
    }

    private RobotState robotState = RobotState.IDLE;

    private Superstructure() {
        super("Superstructure");

        fieldObject2d = field2d.getObject("poses");

        SmartDashboard.putData("superstructure/field", field2d);
    }

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

    public static final Pose2d shooterOffset = new Pose2d(0.0254, 0, Rotation2d.fromDegrees(0));
    // public static final Pose2d shooterOffset = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    @Override
    public void periodic() {
        SwerveDriveState state = drivetrain.getState();
        dtPose = state.Pose;
        Rotation2d dtRotation = dtPose.getRotation();
        shooterPose = new Pose2d(
                dtPose.getX() + dtRotation.getCos() * shooterOffset.getX() - dtRotation.getSin() * shooterOffset.getY(),
                dtPose.getY() + dtRotation.getSin() * shooterOffset.getX() + dtRotation.getCos() * shooterOffset.getY(),
                new Rotation2d()
        );
        SmartDashboard.putBoolean("superstructure/in alliance zone", inAllianceZone());
        boolean isBlue = drivetrain.isBlue();

        Pose2d staticPose = this.inAllianceZone() ? 
            (isBlue ? Localization.blueHub : Localization.redHub) :
            new Pose2d(); // will be lob poses later

        ShotParameter shotParameter =  (this.inAllianceZone()) ? 
                        InterpolatingTable.get(shooterPose.minus(staticPose).getTranslation().getNorm()) :
                        LobInterpolatingTable.get(shooterPose.minus(staticPose).getTranslation().getNorm()); 
                        //calculates exit velocity using static pose
        
        double distanceToHub = shooterPose.getTranslation().getDistance(staticPose.getTranslation());
        double exitVelocity = SOTMCalcs.rpmToExitVelocity(shotParameter.getShooterRPM());

        double[] trueRobotVelocity = SOTMCalcs.rotate(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, dtRotation);

        double r = shooterOffset.getTranslation().getNorm();
        double omega = state.Speeds.omegaRadiansPerSecond;
        Rotation2d tangentialAngle = dtRotation.plus(Rotation2d.fromDegrees(90));

        double[] shooterPoseVelocity = new double[]{
            omega * r * tangentialAngle.getCos(),
            omega * r * tangentialAngle.getSin()
        };

        SmartDashboard.putNumber("auto aim/shooter velo/x", shooterPoseVelocity[0]);
        SmartDashboard.putNumber("auto aim/shooter velo/y", shooterPoseVelocity[1]);

        SmartDashboard.putNumber("auto aim/lerp/rpm", shotParameter.getShooterRPM());
        SmartDashboard.putNumber("auto aim/lerp/deg", shotParameter.pivotAngleDeg);

        SmartDashboard.putNumber("auto aim/exit velocity", exitVelocity);


        Transform2d sotmOffset = new Transform2d(SOTMCalcs.getOffset(
                    trueRobotVelocity[0] + shooterPoseVelocity[0],
                    trueRobotVelocity[1] + shooterPoseVelocity[1],
                    exitVelocity,
                    distanceToHub),
                    new Rotation2d());

        SmartDashboard.putNumber("auto aim/sotm offset/x", sotmOffset.getX());
        SmartDashboard.putNumber("auto aim/sotm offset/y", sotmOffset.getY());

        SmartDashboard.putNumber("auto aim/velo/x", state.Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("auto aim/velo/y", state.Speeds.vyMetersPerSecond);

        SmartDashboard.putNumber("auto aim/true velo/x", trueRobotVelocity[0]);
        SmartDashboard.putNumber("auto aim/true velo/y", trueRobotVelocity[1]);


        this.targetPose = staticPose.transformBy(
                        sotmOffset
                    );

        ShotParameter trueShotParameter =  (this.inAllianceZone()) ? 
                        InterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm()) :
                        LobInterpolatingTable.get(shooterPose.minus(targetPose).getTranslation().getNorm());
                        // actual shot parameter for adjusted sotm pose

        Rotation2d turretTargetAngle = 
                    Rotation2d.fromRadians(
                        Math.atan2(
                            targetPose.getY() - shooterPose.getY(), 
                            targetPose.getX() - shooterPose.getX())
                    ).minus(dtRotation);

        SmartDashboard.putNumberArray("superstructure/target pose", targetPose.toMatrix().getData()); //for tuning, prob dont need for comp
        SmartDashboard.putNumber("superstructure/distance to hub", distanceToHub); //for tuning, prob dont need for comp

        ArrayList<Pose2d> poses = new ArrayList<>();
        poses.add(dtPose);
        poses.add(new Pose2d(shooterPose.getX(), shooterPose.getY(), turretTargetAngle.plus(dtRotation)));
        poses.add(targetPose);
        poses.add(staticPose);
        fieldObject2d.setPoses(poses);

        switch (robotState) {
            case MANUAL_SHOT:
                if (readyToShoot()) index.startIndex();
                break;
            case LERP_TUNING:
                turret.setTurretAngle(
                    turretTargetAngle
                );
                if (readyToShoot()) index.startIndex();
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
                if (!readyToShoot()) setState(RobotState.FULL_TRACKING);
            case FULL_TRACKING:
                shooter.setShotParameter(
                    trueShotParameter
                );

                if (readyToShoot()) setState(RobotState.SHOOTING);
            case TURRET_TRACKING:
                turret.setTurretAngle(
                    turretTargetAngle
                );
                break;
            case IDLE:
                break;
        }

        SmartDashboard.putString("ROBOT STATE", this.robotState.toString());
        SmartDashboard.putBoolean("superstructure/ready to shoot", this.readyToShoot());
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
            case FULL_TRACKING:
                index.stopIndex();
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

    public void setLerpTuneShotParameter(ShotParameter shot) {
        this.shooter.setShotParameter(shot);
        this.setState(RobotState.LERP_TUNING);
    }

    public Command setLerpTuneShotParameterCommand(ShotParameter shot) {
        return Commands.runOnce(() -> setLerpTuneShotParameter(shot), this);
    }

    public Command resetShooterHoodCommand() {
        return shooter.resetHoodCommand();
    }

    public Command intakeSafeStowCommand() {
        return Commands.sequence(
            intake.stopIntakeCommand(),
            index.khangaiIsAChudCommand(),
            Commands.waitUntil(() -> index.inSafePosition()),
            intake.stowIntakeCommand(),
            Commands.waitUntil(() -> intake.atTargetPosition())
        );
    }

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }
}
