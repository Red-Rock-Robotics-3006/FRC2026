package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;
import redrocklib.util.LerpingSmartDashboardNumber;
import redrocklib.wrappers.RedRockTalon;

public class Turret extends SubsystemBase{
    private static boolean kEnableTurretTuning = true;
    private static Turret instance = null;

    public static final int kCcoderAToothCount = 15;
    public static final int kCcoderBToothCount = 16;

    public static final int kLCMToothCount = 240;

    public static final int kTurretToothCount = 88;

    private double targetTurretPositionMotorRotations = 0;

    private SmartDashboardNumber turretTolerance = new SmartDashboardNumber("turret/tolerance", 2, kEnableTurretTuning && true);

    private RedRockTalon turretMotor = new RedRockTalon(44, "turret-motor", "*");

    private CANcoder ccoderA = new CANcoder(45, "*");
    private CANcoder ccoderB = new CANcoder(47, "*");

    private SmartDashboardNumber turretLoomMinAngle = new SmartDashboardNumber("turret/min loom angle", -260);

    private SmartDashboardNumber turretTuningA = new SmartDashboardNumber("turret/tuning/motor pos a", 10);
    private SmartDashboardNumber turretTuningB = new SmartDashboardNumber("turret/tuning/motor pos b", 20);

    private SmartDashboardNumber turretTuningAngle = new SmartDashboardNumber("turret/tuning/rotation2d deg", 0);

    // private SmartDashboardNumber crtTestA = new SmartDashboardNumber("turret/crt/A", 0);
    // private SmartDashboardNumber crtTestB = new SmartDashboardNumber("turret/crt/B", 0);

    private LerpingSmartDashboardNumber turretRestrictions
        = new LerpingSmartDashboardNumber(
            -270, 0, 
            180, 50.1484375, 
            // 270, 110.0/16.0,
            "turret/angle-degrees", "turret/motor-rotations", 
            kEnableTurretTuning && true);

    private Turret() {
        super();

        turretMotor.withTuningEnabled(kEnableTurretTuning && true)
        .withMotorOutputConfigs(
            new MotorOutputConfigs()
            .withPeakForwardDutyCycle(1)
            .withPeakReverseDutyCycle(-1)
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)
        ).withSlot0Configs(
            new Slot0Configs()
            .withKP(6.1)
            .withKS(0.22)
        ).withMotionMagicConfigs(
            new MotionMagicConfigs()
            .withMotionMagicAcceleration(450)
            .withMotionMagicCruiseVelocity(300)
            .withMotionMagicJerk(99999)
        );

        ccoderA.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset(-0.964599609375)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0)
        );

        ccoderB.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset(-0.28662109375)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1.0)
        );

        this.turretMotor.motor.getConfigurator().apply(
            new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        );

        // this.turretMotor.motor.getConfigurator().apply(
        //     new FeedbackConfigs()
        //     .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        //     .withFeedbackRemoteSensorID(ccoderB.getDeviceID())
        // );

        this.calibrateTurret();
        // this.turretMotor.resetMotor();
    }

    public void calibrateTurret() {
        // this.turretMotor.resetMotor();
        this.turretMotor.motor.setControl(new CoastOut());
        double crtDeg;
        try {
            crtDeg = this.crtDegrees();
        } catch (Exception e) {
            return;
        }

        if (crtDeg < turretRestrictions.getMinInput() || crtDeg > turretRestrictions.getMaxInput()) return;
        this.turretMotor.motor.setPosition(
            turretRestrictions.getValue(
                crtDeg
            )
        );
    }

    private double crtDegrees() {
        double e1 = ccoderA.getAbsolutePosition().getValueAsDouble();
        if (e1 < 0) e1 = 1 - e1;
        double e2 = ccoderB.getAbsolutePosition().getValueAsDouble();
        if (e2 < 0) e2 = 1 - e2;

        double tooth1 = e1 * kCcoderAToothCount;
        double remainder = tooth1 - (int)(tooth1);
        double tooth2 = e2 * kCcoderBToothCount;

        double absTurretToothCount = TurretCalcs.solveCRT((int)tooth1, (int)tooth2) + remainder;

        // tooth2 = (int)(tooth2) + remainder;
        // double absTurretToothCount = TurretCalcs.solveCRT(tooth1, tooth2);

        double absDegrees = (absTurretToothCount / kTurretToothCount) * 360 + turretRestrictions.getMinInput();

        return absDegrees;
    }
    
    @SuppressWarnings("unused")
    private double crtDegrees(double e1, double e2) {
        // double e1 = ccoderA.getAbsolutePosition().getValueAsDouble();
        if (e1 < 0) e1 = 1 - e1;
        // double e2 = ccoderB.getAbsolutePosition().getValueAsDouble();
        if (e2 < 0) e2 = 1 - e2;

        double tooth1 = e1 * kCcoderAToothCount;
        double remainder = tooth1 - (int)(tooth1);
        double tooth2 = e2 * kCcoderBToothCount;

        double absTurretToothCount = TurretCalcs.solveCRT((int)tooth1, (int)tooth2) + remainder;

        // tooth2 = (int)(tooth2) + remainder;
        // double absTurretToothCount = TurretCalcs.solveCRT(tooth1, tooth2);

        double absDegrees = (absTurretToothCount / kTurretToothCount) * 360 + turretRestrictions.getMinInput();

        return absDegrees;
    }

    /**
     * Sets the target angle of the turret.
     * 
     * @param angle Desired angle of turret. 0 is facing forward on the robot, CCW+
     */
    public void setTurretAngle(Rotation2d angle) {
        double currentAngle = this.getTruePositionDegrees();
        double targetDeg = angle.getDegrees();

        double dist = ((targetDeg - currentAngle + 180) % 360 + 360) % 360 - 180;
        targetDeg = currentAngle + dist;

        if (targetDeg < turretLoomMinAngle.getNumber()) targetDeg += 360;
        else if (targetDeg > turretRestrictions.getMaxInput()) targetDeg -= 360;

        this.setTurretPosition(
            turretRestrictions.getValue(targetDeg)
        );

        // SmartDashboard.putNumber("Turret/rot debug/target", targetDeg);
        // SmartDashboard.putNumber(getName(), dist);
        // SmartDashboard.putNumber("Turret/rot debug/dist", dist);
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
                turretRestrictions.getValue(turretLoomMinAngle.getNumber()), 
                turretRestrictions.getMaxOutput())
        );
        this.targetTurretPositionMotorRotations = position;
    }

    public boolean atTurretAngle() {
        return Math.abs(turretMotor.motor.getPosition().getValueAsDouble() - this.targetTurretPositionMotorRotations)
            < Math.abs(turretRestrictions.convertOutputByRate(turretTolerance.getNumber()));
    }

    public double getTruePositionDegrees() {
        return turretRestrictions.getValueInverse(
                this.turretMotor.motor.getPosition().getValueAsDouble()
            );
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(
            this.getTruePositionDegrees()
        );
    }

    public void setTuningPosA() {
        this.setTurretPosition(turretTuningA.getNumber());
    }

    public void setTuningPosB() {
        this.setTurretPosition(turretTuningB.getNumber());
    }

    public void setTuningRotation() {
        this.setTurretAngle(Rotation2d.fromDegrees(turretTuningAngle.getNumber()));
    }

    @Override
    public void periodic() {
        turretMotor.update();

        if (DriverStation.isDisabled()) this.calibrateTurret();

        SmartDashboard.putNumber("turret/motor degrees", this.getTruePositionDegrees());
        SmartDashboard.putNumber("turret/CRT degrees", this.crtDegrees());

        SmartDashboard.putNumber("turret/ccoder A", this.ccoderA.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("turret/ccoder B", this.ccoderB.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("turret/target motor rotations", this.targetTurretPositionMotorRotations);

        // SmartDashboard.putNumber("turret/crt/result", this.crtDegrees(crtTestA.getNumber() / 16.0, crtTestB.getNumber() / 15.0));
    }

    public static class TurretCalcs {
        /**
         * pure double crt is slow and bad for loop times
         * use int crt and then add the double to the result
         * @param e1
         * @param e2
         * @return
         */
        @Deprecated
        public static double solveCRT(double e1, double e2) {
            int m1 = kCcoderAToothCount, m2 = kCcoderBToothCount;
            @SuppressWarnings("unused")
            double M = m1 * m2; // 240.0

            // Brute-force search over k1: x = e1 + 15*k1, check if (x - e2) divisible by 16
            double epsilon = 1e-9;
            int maxIter = 100000;

            for (int k1 = 0; k1 < maxIter; k1++) {
                double x = e1 + m1 * k1;
                if (x <= 0) continue;
                double remainder = (x - e2) / m2;
                if (Math.abs(remainder - Math.round(remainder)) < epsilon) {
                    return x;
                }
            }
            // return 1000000;
            throw new ArithmeticException("No solution found within iteration limit");
        }

        public static int solveCRT(int e1, int e2) {
            int m1 = kCcoderAToothCount, m2 = kCcoderBToothCount;
            int M = m1 * m2; // 240

            // Modular inverse of 15 mod 16 (since 15 * 15 = 225 = 14*16 + 1, inv = 15)
            int inv_m1 = modInverse(m1, m2);

            int x = (e1 + m1 * (Math.floorMod((e2 - e1) * inv_m1, m2))) % M;
            return x == 0 ? M : x;
        }

        private static int modInverse(int a, int m) {
            // Extended Euclidean Algorithm
            int m0 = m, x0 = 0, x1 = 1;
            if (m == 1) return 0;
            while (a > 1) {
                int q = a / m;
                int t = m;
                m = a % m;
                a = t;
                t = x0;
                x0 = x1 - q * x0;
                x1 = t;
            }
            return x1 < 0 ? x1 + m0 : x1;
        }
    }
    
    public static Turret getInstance() {
        if (instance == null) instance = new Turret();
        return instance;
    }
}
