package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class FMSMatchTimer extends PracticeTimer{
    @Override
    protected double getInMatchTime() {
        if (DriverStation.isTeleopEnabled()) {
            return 140d - Timer.getMatchTime();
        }
        return 0;
    }
}
