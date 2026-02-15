package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Superstructure;

import redrocklib.logging.SmartDashboardNumber;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(9); //TODO
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(300); //TODO
    
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

    private final Superstructure superstructure = Superstructure.getInstance();

    private LED() {
        super();
        
        this.control.setLength(this.buffer.getLength());
        this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);

        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        
        this.control.start();
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
    
    @Override
    public void periodic() {
        blinkControl++;

        switch (superstructure.getRobotState()) {
            case MANUAL_SHOT:
                this.setLights(BLUE);
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
                this.setLights(GREEN);
                break;
            case FULL_TRACKING:
                this.blink(RED, 4);
                break;
            case TURRET_TRACKING:
                this.rainbow();
                break;
            case IDLE:
                this.rainbow();
                break;
        }

        this.control.setData(buffer);
    }

    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}