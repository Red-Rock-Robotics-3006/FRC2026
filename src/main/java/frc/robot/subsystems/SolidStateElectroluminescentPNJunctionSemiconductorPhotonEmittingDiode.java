package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class SolidStateElectroluminescentPNJunctionSemiconductorPhotonEmittingDiode extends SubsystemBase{

    private static SolidStateElectroluminescentPNJunctionSemiconductorPhotonEmittingDiode instance = null;

    private AddressableLED control = new AddressableLED(9); //TODO
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(300); //TODO
    private SmartDashboardNumber rainbowControl = new SmartDashboardNumber("led/rainbow control", 3);

    private final Color INIT_YELLOW = new Color(255, 165, 0);

    public final Color OFF = new Color(0, 0, 0);
    public final Color GREEN = new Color(0, 255, 0);
    public final Color MAGENTA = new Color(255, 0, 255);
    public final Color NOTE_ORANGE = new Color(255, 15, 0);
    public final Color WHITE = new Color(255, 255, 255);
    public final Color BLUE = new Color(0, 0, 255);
    public final Color RED = new Color(255, 0, 0);

    private int blinkControl = 0;

    private SolidStateElectroluminescentPNJunctionSemiconductorPhotonEmittingDiode() {
        super();
        
        this.control.setLength(this.buffer.getLength());
        this.control.setColorOrder(AddressableLED.ColorOrder.kRGB);

        this.setLights(INIT_YELLOW);
        this.control.setData(buffer);
        
        this.control.start();
    }

    public void setLights(int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = 0; i < buffer.getLength(); i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(int start, int end, int r, int g, int b) {
        if (r > 255 || g > 255 || b > 255) {
            for (int i = start; i < end; i++) {
                this.buffer.setRGB(i, 255, 255, 255);
            }
        }
        else {
            for (int i = start; i < end; i++) {
                this.buffer.setRGB(i, r, g, b);
            }
        }
    }

    public void setLights(Color c) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, c);
        }
    }

    public void setLights(int start, int end, Color c) {
        for (int i = start; i < end; i++) {
            buffer.setLED(i, c);
        }
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
    
    @Override
    public void periodic() {
        blinkControl++;
        this.control.setData(buffer);
    }

    public static SolidStateElectroluminescentPNJunctionSemiconductorPhotonEmittingDiode getInstance(){
        if (instance == null) instance = new SolidStateElectroluminescentPNJunctionSemiconductorPhotonEmittingDiode();
        return instance;
    }
}