package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.autoaim.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain.DriveState;
import frc.robot.subsystems.vision.Localization;

import redrocklib.logging.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance = null;

    public static final boolean kPractice = false;
    public static final boolean kHubOrLob = true; //true for hub, false for lob
    public static final boolean kTuning = true;

    public final Intake intake = Intake.getInstance();
    public final Index index = Index.getInstance();
    public final Shooter shooter = Shooter.getInstance();
    public final Turret turret = Turret.getInstance(); //private ts for comp
    public final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public final Localization localization = Localization.getInstance();

    private final double blueTrenchX = Units.inchesToMeters(182.11);
    private final double redTrenchX = Units.inchesToMeters(469.11);
    private final double midfieldX = Units.inchesToMeters(325.61);
    private final double midfieldY = Units.inchesToMeters(158.845);
    
    private SmartDashboardNumber lobDisableZoneUpperY = new SmartDashboardNumber("superstructure/lob disable zone upper y", 4.5, kTuning);
    private SmartDashboardNumber lobDisableZoneLowerY = new SmartDashboardNumber("superstructure/lob disable zone lower y", 3.44, kTuning);
    private SmartDashboardNumber trenchZoneTolerance = new SmartDashboardNumber("superstructure/trench zone tolerance", 0.5, kTuning); //tolerance for hood near trench, probably best to be generous with this one
    private SmartDashboardBoolean trenchSafetyEnabled = new SmartDashboardBoolean("superstructure/trench safety enabled", true);
    // private SmartDashboardNumber indexJamReverseTime = new SmartDashboardNumber("index/index jam reverse time", 0.5);

    private SmartDashboardNumber autoAimOffsetRPM = new SmartDashboardNumber("superstructure/auto aim offset rpm", 0);
    private SmartDashboardNumber autoAimOffsetHood = new SmartDashboardNumber("superstructure/auto aim offset hood", 0);

    private final ShotParameter SHOOTER_IDLE_PARAMETER = new ShotParameter(12.7, 0);
    private final ShotParameter SHOOTER_REVERSE_PARAMETER = new ShotParameter(12.7, -700);

    private SmartDashboardNumber dtRotationTurretOffsetCoeff = new SmartDashboardNumber("sotm/rotation fudge factor", 0.1);

    private Field2d field2d = new Field2d();
    private FieldObject2d fieldObject2d = field2d.getObject("poses");

    public enum RobotState {
        TURRET_TRACKING, //turret tracking
        FULL_TRACKING, //flywheels spin up, hood tracking, turret tracking
        SHOOTING, //index spinning, flywheels spinning, hood tracking, turret tracking
        SHOOTING_JAMMED, //index jammed, flywheels spinning, hood tracking, turret tracking

        MANUAL_SHOT, //any manual shot, turret at set angle, hood at set angle, flywheels at set rpm
        LERP_TUNING, //for tuning lerp tables, turret tracking, hood at set angle, flywheels at set rpm

        NEAR_TRENCH, //robot near trench, all mechanisms idle
        REVERSE, //index reversing, flywheels reversing
        IDLE //all mechanisms idle
    }

    private RobotState robotState = RobotState.IDLE;
    private RobotState lastState = RobotState.IDLE;

    private Superstructure() {
        super("Superstructure");
        SmartDashboard.putData("superstructure/field", field2d);
    }

    private boolean inAllianceZone() {
        if (kPractice) return kHubOrLob;
        return drivetrain.isBlue() ? shooterPose.getX() < blueTrenchX : shooterPose.getX() > redTrenchX;
    }

    private boolean inLobEnabledZone() {
        return (drivetrain.isBlue() ? shooterPose.getX() > midfieldX : shooterPose.getX() < midfieldX)
            || (shooterPose.getY() > lobDisableZoneUpperY.getNumber() || shooterPose.getY() < lobDisableZoneLowerY.getNumber());
    }

    private boolean inLobUpperZone() {
        return shooterPose.getY() > midfieldY;
    }

    private boolean nearTrench() {
        return (shooterPose.getX() > blueTrenchX - trenchZoneTolerance.getNumber() && shooterPose.getX() < blueTrenchX + trenchZoneTolerance.getNumber()) ||
            (shooterPose.getX() > redTrenchX - trenchZoneTolerance.getNumber() && shooterPose.getX() < redTrenchX + trenchZoneTolerance.getNumber());
    }

    private boolean readyToShoot() {
        return shooter.atHoodAngle() && shooter.atShooterSpeed() && turret.atTurretAngle();
    }

    // private Timer indexTimer = new Timer();

    private Pose2d dtPose;
    private Pose2d shooterPose;
    private Pose2d dynamicTargetPose = new Pose2d();

    public static final Pose2d shooterOffset = new Pose2d(0.0254, 0, Rotation2d.fromDegrees(0));

    @Override
    public void periodic() {
        SwerveDriveState state = drivetrain.getState();
        Localization.setCameraDynamicRotation(shooterOffset.getTranslation(), turret.getRotation(), state.Pose.getRotation());
        boolean isBlue = drivetrain.isBlue();
        dtPose = state.Pose;

        Rotation2d dtRotation = dtPose.getRotation();
        shooterPose = new Pose2d(
            dtPose.getX() + dtRotation.getCos() * shooterOffset.getX() - dtRotation.getSin() * shooterOffset.getY(),
            dtPose.getY() + dtRotation.getSin() * shooterOffset.getX() + dtRotation.getCos() * shooterOffset.getY(),
            new Rotation2d()
        );

        drivetrain.setInLobZone(!this.inAllianceZone());

        Pose2d staticTargetPose = 
            this.inAllianceZone() ? 
                (isBlue ? Localization.blueHub : Localization.redHub) :
                (isBlue ? 
                    (inLobUpperZone() ? Localization.blueLobTargets[0] : Localization.blueLobTargets[1]) : 
                    (inLobUpperZone() ? Localization.redLobTargets[0] : Localization.redLobTargets[1]));

        ShotParameter staticShotParameter =  (this.inAllianceZone()) ? 
            HubInterpolatingTable.get(shooterPose.minus(staticTargetPose).getTranslation().getNorm()) :
            LobInterpolatingTable.get(shooterPose.minus(staticTargetPose).getTranslation().getNorm()); 
            //calculates exit velocity using static pose
        
        double distanceToTarget = shooterPose.getTranslation().getDistance(staticTargetPose.getTranslation());
        double exitVelocity = SOTMCalcs.rpmToExitVelocity(staticShotParameter.getShooterRPM());

        double[] fieldCentricRobotVelocity = SOTMCalcs.rotate(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond, dtRotation);

        double r = shooterOffset.getTranslation().getNorm();
        double omega = state.Speeds.omegaRadiansPerSecond;
        Rotation2d tangentialAngle = dtRotation.plus(Rotation2d.fromDegrees(90));

        double[] shooterPoseVelocity = new double[]{
            omega * r * tangentialAngle.getCos(),
            omega * r * tangentialAngle.getSin()
        };

        Transform2d sotmOffset = new Transform2d(SOTMCalcs.getSecantOffset(
            fieldCentricRobotVelocity[0] + shooterPoseVelocity[0],
            fieldCentricRobotVelocity[1] + shooterPoseVelocity[1],
            staticTargetPose,
            dtPose),
            new Rotation2d());

        this.dynamicTargetPose = staticTargetPose.transformBy(sotmOffset);


        // ACTUAL SHOT PARAMETER FOR ADJUSTED SOTM POSE
        ShotParameter dynamicShotParameter =  (this.inAllianceZone()) ? 
            HubInterpolatingTable.get(shooterPose.minus(dynamicTargetPose).getTranslation().getNorm()) :
            LobInterpolatingTable.get(shooterPose.minus(dynamicTargetPose).getTranslation().getNorm());
        
        dynamicShotParameter = new ShotParameter(dynamicShotParameter.pivotAngleDeg + autoAimOffsetHood.getNumber(), dynamicShotParameter.rpm + autoAimOffsetRPM.getNumber());

        // ACTUAL TURRET ANGLE FOR ADJUSTED SOTM POSE
        Rotation2d turretTargetAngle = 
            Rotation2d.fromRadians(
                Math.atan2(
                    dynamicTargetPose.getY() - shooterPose.getY(), 
                    dynamicTargetPose.getX() - shooterPose.getX())
            ).minus(dtRotation);

        // TURRET COMPENSATION FOR SWERVE ROTATION
        Rotation2d dtTurretCompensation = 
            Rotation2d.fromDegrees(
                -dtRotationTurretOffsetCoeff.getNumber() * 
                Math.toDegrees(state.Speeds.omegaRadiansPerSecond)
            );
        

        switch (robotState) {
            case SHOOTING_JAMMED:
                // if (indexTimer.get() > indexJamReverseTime.getNumber()) setState(lastState);
                if (!index.isJamming()) setState(lastState);
            case SHOOTING:
                if (!turret.atTurretAngle()) setState(RobotState.FULL_TRACKING);
            case FULL_TRACKING:
                if (index.isJamming()) setState(RobotState.SHOOTING_JAMMED);
                if (!inAllianceZone() && !inLobEnabledZone()) {shooter.setShotParameter(SHOOTER_IDLE_PARAMETER);}
                else {shooter.setShotParameter(dynamicShotParameter);}
                if (this.robotState != RobotState.SHOOTING_JAMMED && this.readyToShoot()) setState(RobotState.SHOOTING);
            case TURRET_TRACKING:
                turret.setTurretAngle(turretTargetAngle.plus(dtTurretCompensation));
                break;
            
            case MANUAL_SHOT:
                if (index.isJamming()) setState(RobotState.SHOOTING_JAMMED);
                turret.setTurretAngle(Rotation2d.fromDegrees(90));
                if (readyToShoot()) index.startIndex();
                break;
            case LERP_TUNING:
                if (index.isJamming()) setState(RobotState.SHOOTING_JAMMED);
                turret.setTurretAngle(turretTargetAngle.plus(dtTurretCompensation));
                if (readyToShoot()) index.startIndex();
                break;
            case NEAR_TRENCH:
                if (!nearTrench() && trenchSafetyEnabled.getValue()) {
                    if (lastState == RobotState.SHOOTING) setState(RobotState.FULL_TRACKING);
                    setState(lastState);
                }
                break;
            
            case REVERSE:
                break;
            case IDLE:
                break;
        }

        if (this.robotState != RobotState.NEAR_TRENCH && nearTrench() && trenchSafetyEnabled.getValue() && drivetrain.getDriveState() != DriveState.AUTO) {setState(RobotState.NEAR_TRENCH);}


        // SMARTDASHBOARD LOGGING | COMPETITION

        SmartDashboard.putString("ROBOT STATE", this.robotState.toString());
        SmartDashboard.putString("LAST STATE", this.lastState.toString());
        SmartDashboard.putBoolean("superstructure/ready to shoot", this.readyToShoot());
        SmartDashboard.putBoolean("superstructure/in alliance zone", inAllianceZone());
            
        SmartDashboard.putNumberArray("superstructure/target pose", dynamicTargetPose.toMatrix().getData());
        SmartDashboard.putNumber("superstructure/distance to target", distanceToTarget);
        SmartDashboard.putBoolean("superstructure/near trench", nearTrench());

        // SMARTDASHBOARD LOGGING | TUNING

        if (kTuning) {
            ArrayList<Pose2d> poses = new ArrayList<>();
            poses.add(dtPose);
            poses.add(new Pose2d(shooterPose.getX(), shooterPose.getY(), turretTargetAngle.plus(dtRotation)));
            poses.add(dynamicTargetPose);
            poses.add(staticTargetPose);
            fieldObject2d.setPoses(poses);

            SmartDashboard.putBoolean("superstructure/in lob enabled zone", this.inLobEnabledZone());
            
            SmartDashboard.putNumber("auto aim/sotm offset/x", sotmOffset.getX());
            SmartDashboard.putNumber("auto aim/sotm offset/y", sotmOffset.getY());

            SmartDashboard.putNumber("auto aim/velo/x", state.Speeds.vxMetersPerSecond);
            SmartDashboard.putNumber("auto aim/velo/y", state.Speeds.vyMetersPerSecond);

            SmartDashboard.putNumber("auto aim/true velo/x", fieldCentricRobotVelocity[0]);
            SmartDashboard.putNumber("auto aim/true velo/y", fieldCentricRobotVelocity[1]);
            
            SmartDashboard.putNumber("auto aim/shooter velo/x", shooterPoseVelocity[0]);
            SmartDashboard.putNumber("auto aim/shooter velo/y", shooterPoseVelocity[1]);

            SmartDashboard.putNumber("auto aim/lerp/rpm", staticShotParameter.getShooterRPM());
            SmartDashboard.putNumber("auto aim/lerp/deg", staticShotParameter.pivotAngleDeg);

            SmartDashboard.putNumber("auto aim/dynamic lerp/rpm", dynamicShotParameter.getShooterRPM());
            SmartDashboard.putNumber("auto aim/dynamic lerp/deg", dynamicShotParameter.pivotAngleDeg);

            SmartDashboard.putNumber("auto aim/exit velocity", exitVelocity);
        }
    }

    public void setState(RobotState state) {
        this.lastState = this.robotState;
        if (state == RobotState.FULL_TRACKING || state == RobotState.SHOOTING) drivetrain.enableSpeedLimiter();
        else drivetrain.disableSpeedLimiter();
        switch (state) {
            case IDLE:
                index.stopIndex();
                shooter.setShotParameter(SHOOTER_IDLE_PARAMETER);
                break;
            case SHOOTING:
                index.startIndex();
                break;
            case FULL_TRACKING:
                index.stopIndex();
                break;
            case TURRET_TRACKING:
                index.stopIndex();
                shooter.setShotParameter(SHOOTER_IDLE_PARAMETER);
                break;
            case REVERSE:
                index.reverseIndex();
                shooter.setShotParameter(SHOOTER_REVERSE_PARAMETER);
                break;
            case NEAR_TRENCH:
                index.stopIndex();
                shooter.setShotParameter(SHOOTER_IDLE_PARAMETER);
                break;
            case SHOOTING_JAMMED:
                // indexTimer.reset();
                // index.reverseIndex();
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

    public Command setManualShotParameterCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.shooter.setManualShot(), this, this.shooter),
            Commands.runOnce(() -> this.setState(RobotState.MANUAL_SHOT))
        );
    }

    public Command setLerpTuneParameterCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> this.shooter.setLerpTuneShot(), this, this.shooter),
            Commands.runOnce(() -> this.setState(RobotState.LERP_TUNING))
        );
    }

    public Command resetSuperStructure() {
        return Commands.parallel(
            Commands.runOnce(() -> drivetrain.enableIgnoreCameraDistance(), drivetrain),
            Commands.runOnce(() -> turret.calibrateTurret(), turret)//,
            // shooter.resetHoodCommand()
        );
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

    public Command toggleTrenchSafetyCommand() {
        return Commands.runOnce(() -> trenchSafetyEnabled.putBoolean(!trenchSafetyEnabled.getValue()));
    }

    public void increaseAutoAimRPMOffset() {
        this.autoAimOffsetRPM.putNumber(this.autoAimOffsetRPM.getNumber() + 25);
    }

    public void decreaseAutoAimRPMOffset() {
        this.autoAimOffsetRPM.putNumber(this.autoAimOffsetRPM.getNumber() - 25);
    }

    public void increaseAutoAimHoodOffset() {
        this.autoAimOffsetHood.putNumber(this.autoAimOffsetHood.getNumber() + 1);
    }

    public void decreaseAutoAimHoodOffset() {
        this.autoAimOffsetHood.putNumber(this.autoAimOffsetHood.getNumber() - 1);
    }

    public Command increaseAutoAimRPMOffsetCommand() {
        return Commands.runOnce(() -> increaseAutoAimRPMOffset());
    }

    public Command decreaseAutoAimRPMOffsetCommand() {
        return Commands.runOnce(() -> decreaseAutoAimRPMOffset());
    }

    public Command increaseAutoAimHoodOffsetCommand() {
        return Commands.runOnce(() -> increaseAutoAimHoodOffset());
    }

    public Command decreaseAutoAimHoodOffsetCommand() {
        return Commands.runOnce(() -> decreaseAutoAimHoodOffset());
    }

    public static Superstructure getInstance() {
        if (instance == null) instance = new Superstructure();
        return instance;
    }
}
