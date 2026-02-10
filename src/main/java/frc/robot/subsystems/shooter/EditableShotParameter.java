package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import redrocklib.logging.SmartDashboardNumber;

public class EditableShotParameter extends ShotParameter {
    private SmartDashboardNumber pivotAngleDegSD;// = new SmartDashboardNumber("editable shot parameter/pivot angle degrees", 0);
    private SmartDashboardNumber rpmSD;// = new SmartDashboardNumber("editable shot parameter/rpm", 0);

    public EditableShotParameter(double pivotAngleDeg, double rpm, String name) {
        super(pivotAngleDeg, rpm);
        pivotAngleDegSD = new SmartDashboardNumber(name + "/pivot angle degrees", pivotAngleDeg);
        rpmSD = new SmartDashboardNumber(name + "/rpm", rpm);
    }

    public void setPivotAngleDeg(double pivotAngleDeg) {
        this.pivotAngleDegSD.putNumber(pivotAngleDeg);
    }

    public void setRPM(double rpm) {
        this.rpmSD.putNumber(rpm);
    }

    @Override
    public Rotation2d getHoodAngle() {
        return Rotation2d.fromDegrees(this.pivotAngleDegSD.getNumber());
    }

    @Override
    public double getShooterRPM() {
        return this.rpmSD.getNumber();
    }
}
