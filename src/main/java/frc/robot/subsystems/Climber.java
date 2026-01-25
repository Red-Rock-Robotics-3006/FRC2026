package frc.robot.subsystems;

public class Climber {
    private static Climber instance = null;

    private Climber() {
        
    }
    
    public static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }
}
