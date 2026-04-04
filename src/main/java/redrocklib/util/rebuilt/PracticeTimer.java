package redrocklib.util.rebuilt;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PracticeTimer {
    public static final boolean kPractice = true;
    private static PracticeTimer instance = null;

    private double matchTime;
    private double shiftTimeLeft;

    protected Timer timer;

    private MatchState matchState = MatchState.MATCH_END;

    public enum MatchState {
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        MATCH_END
    }

    public PracticeTimer() {
        timer = new Timer();
    }

    public void reset() {
        timer.reset();
        timer.start();
    }

    public void stop() {
        this.timer.stop();
        this.matchState = MatchState.MATCH_END;
    }

    protected double getInMatchTime() {
        return timer.get();
    }

    public void update() {
        this.matchTime = this.getInMatchTime();

        Alliance alliance = (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : Alliance.Blue;
        String gameStr = DriverStation.getGameSpecificMessage();

        char autoWinner = (gameStr.isEmpty()) ? 'B' : gameStr.charAt(0);

        boolean winAauto = 
            autoWinner == 'R' && alliance == Alliance.Red ||
            autoWinner == 'B' && alliance == Alliance.Blue;

        String displayString = "";

        if (DriverStation.isDisabled()) {
            this.matchState = MatchState.MATCH_END;
            displayString = "MATCH_END";
        } else if (matchTime < 10) {
            this.shiftTimeLeft = 10d - this.matchTime;
            this.matchState = MatchState.TRANSITION;
            displayString = "TRANS-" + ((winAauto) ? "WON" : "LOST");
        } else if (matchTime < 10d + 25) {
            this.shiftTimeLeft = 10d + 25 - this.matchTime;
            this.matchState = MatchState.SHIFT_1;
            displayString = ((winAauto) ? "LOBBING" : "SHOOTING") + " 1/4";
        } else if (matchTime < 10d + 2 * 25) {
            this.shiftTimeLeft = 10d + 2 * 25 - this.matchTime;
            this.matchState = MatchState.SHIFT_2;
            displayString = ((!winAauto) ? "LOBBING" : "SHOOTING") + " 2/4";
        } else if (matchTime < 10d + 3 * 25) {
            this.shiftTimeLeft = 10d + 3 * 25 - this.matchTime;
            this.matchState = MatchState.SHIFT_3;
            displayString = ((winAauto) ? "LOBBING" : "SHOOTING") + " 3/4";
        } else if (matchTime < 10d + 4 * 25) {
            this.shiftTimeLeft = 10d + 4 * 25 - this.matchTime;
            this.matchState = MatchState.SHIFT_4;
            displayString = ((!winAauto) ? "LOBBING" : "SHOOTING") + " 4/4";
        } else if (matchTime < 10d + 4*25 + 30) {
            this.shiftTimeLeft = 10d + 4*25 + 30 - this.matchTime;
            this.matchState = MatchState.ENDGAME;
            displayString = "ENDGAME";
        } else {
            this.shiftTimeLeft = 0;
            this.matchState = MatchState.MATCH_END;
            displayString = "MATCH_END";
        }

        Logger.recordOutput("Match timer/match time", matchTime);
        Logger.recordOutput("Match timer/shift time", this.shiftTimeLeft);
        SmartDashboard.putString("Match timer/Shift time left", String.format("%.1f", this.shiftTimeLeft));
        SmartDashboard.putString("Match timer/Match time left", toFormattedTime((int)(140d - this.matchTime)));
        SmartDashboard.putString("Match timer/Current shift", displayString);
        Logger.recordOutput("Match timer/match state", this.matchState.toString());
    }

    private static String toFormattedTime(int seconds) {
        String lead = (seconds < 0) ? "-" : "";
        seconds = Math.abs(seconds);
        return lead + String.valueOf(seconds / 60) + ":" + String.valueOf((seconds % 60) / 10) + String.valueOf(seconds % 10);
    }

    public static PracticeTimer getInstance() {
        if (instance == null) instance = (kPractice) ? new PracticeTimer() : new FMSMatchTimer();
        return instance;
    }
}
