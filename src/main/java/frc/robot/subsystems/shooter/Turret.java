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

    public static final int kCcoderAToothCount = 15;
    public static final int kCcoderBToothCount = 16;

    public static final int kLCMToothCount = 240;

    public static final int kTurretToothCount = 88;

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
        this.turretMotor.resetMotor();
        this.turretMotor.motor.setPosition(
            turretRestrictions.getValue(
                this.crtDegrees()
            )
        );
    }

    private double crtDegrees() {
        double e1 = ccoderA.getPosition().getValueAsDouble();
        double e2 = ccoderB.getPosition().getValueAsDouble();

        double tooth1 = e1 * kCcoderAToothCount;
        double remainder = tooth1 - (int)(tooth1);
        double tooth2 = e2 * kCcoderBToothCount;
        tooth2 = (int)(tooth2) + remainder;

        double absTurretToothCount = TurretCalcs.solveCRT(tooth1, tooth2);

        double absDegrees = (absTurretToothCount / kTurretToothCount) * 360 + turretRestrictions.getMinInput();

        return absDegrees;
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

    public static class TurretCalcs {
        public static double solveCRT(double e1, double e2) {
            int m1 = kCcoderAToothCount, m2 = kCcoderBToothCount;
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

            throw new ArithmeticException("No solution found within iteration limit");
        }

        public static int solveCRT(int e1, int e2) {
            int m1 = 15, m2 = 16;
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
