package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(0); //TODO
    private AddressableLEDSim controlSim; //TODO
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(50); //TODO
    
    public final Color INIT_YELLOW = new Color(255, 165, 0);
    public final Color OFF = new Color(0, 0, 0);
    public final Color GREEN = new Color(0, 255, 0);
    public final Color MAGENTA = new Color(255, 0, 255);
    public final Color NOTE_ORANGE = new Color(255, 15, 0);
    public final Color WHITE = new Color(255, 255, 255);
    public final Color BLUE = new Color(0, 0, 255);
    public final Color RED = new Color(255, 0, 0);
    
    private int blinkControl = 0;
    private SmartDashboardNumber rainbowControl = new SmartDashboardNumber("led/rainbow control", 3);

    private LED() {
        super("LED");
        
        this.control.setLength(this.buffer.getLength());
        this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);

        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        
        this.control.start();

        controlSim = new AddressableLEDSim(control);
        controlSim.setInitialized(true);
    }

    public enum RobotState {
        MANUAL_SHOT, SHOOTING_WHILE_MOVING, SHOOTING, FULL_TRACKING, TURRET_TRACKING, IDLE
    }

    private RobotState robotState = RobotState.IDLE;

    public void setRobotState(RobotState state) {
        this.robotState = state;
        Commands.print("LED state set to " + state.toString());
    }

    public RobotState getRobotState() {
        return this.robotState;
    }

    public void setLights(Color c) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, c);
        }
    }

    public void blink(Color c, int freq) {
        if (blinkControl % freq * 2 < freq) this.setLights(c);
        else this.setLights(OFF);
    }

    public void blinkSetLights(Color c, int blinks, int freq) {
        if (blinkControl < freq * blinks) {
            if (blinkControl % freq * 2 < freq) this.setLights(c);
            else this.setLights(OFF);
        }
        else setLights(c);
    }

    int rainbowHue = 0;
    public void rainbow() {
        for (var i = 0; i < buffer.getLength(); i++) {
          final var hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        rainbowHue += rainbowControl.getNumber();
        rainbowHue %= 180;
    }

    public void increaseHueControl() {rainbowControl.putNumber(rainbowControl.getNumber() + 1);}
    public void decreaseHueControl() {rainbowControl.putNumber(rainbowControl.getNumber() - 1);}

    private int larsonPosition = 0;
    private SmartDashboardNumber larsonSpeed = new SmartDashboardNumber("led/larson speed", 2);
    private int larsonDirection = (int) larsonSpeed.getNumber();
    private final int LARSON_SIZE = 15;

    public void larson(Color c) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, new Color(
                buffer.getLED(i).red * 0.5,
                buffer.getLED(i).green * 0.5,
                buffer.getLED(i).blue * 0.5
            ));
        }

        for (int offset = -(LARSON_SIZE / 2); offset <= LARSON_SIZE / 2; offset++) {
            int index = larsonPosition + offset;
            if (index < 0 || index >= buffer.getLength()) continue;

            double brightness = 1.0 - (Math.abs(offset) / (double)(LARSON_SIZE / 2 + 1));

            buffer.setLED(index, new Color(
                c.red * brightness,
                c.green * brightness,
                c.blue * brightness
            ));
        }

        larsonPosition += larsonDirection;
        if (larsonPosition >= buffer.getLength() - 1) {
            larsonPosition = buffer.getLength() - 1;
            larsonDirection = (int) -larsonSpeed.getNumber();
        } else if (larsonPosition <= 0) {
            larsonPosition = 0;
            larsonDirection = (int) larsonSpeed.getNumber();
        }
    }
    
    @Override
    public void periodic() {
        blinkControl++;

        switch (this.getRobotState()) {
            case MANUAL_SHOT:
                this.setLights(BLUE);
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
                this.setLights(GREEN);
                break;
            case FULL_TRACKING:
                this.blink(RED, 8);
                break;
            case TURRET_TRACKING:
                this.larson(GREEN);
                break;
            case IDLE:
                this.larson(WHITE);
                break;
        }

        this.control.setData(buffer);

        SmartDashboard.putString("ROBOT STATE", this.getRobotState().toString());
    }

    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}