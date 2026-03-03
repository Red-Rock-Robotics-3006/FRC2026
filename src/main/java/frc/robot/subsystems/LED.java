package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import redrocklib.logging.SmartDashboardNumber;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(0); //TODO
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(150); //TODO

    private AddressableLEDBufferView left = buffer.createView(0, 49);
    private AddressableLEDBufferView back = buffer.createView(50, 99);
    private AddressableLEDBufferView right = buffer.createView(100, 149);
    
    public final Color INIT_YELLOW = new Color(255, 165, 0);
    public final Color OFF = new Color(0, 0, 0);
    public final Color GREEN = new Color(0, 255, 0);
    public final Color MAGENTA = new Color(255, 0, 255);
    public final Color NOTE_ORANGE = new Color(255, 165, 0);
    public final Color WHITE = new Color(255, 255, 255);
    public final Color BLUE = new Color(0, 0, 255);
    public final Color RED = new Color(255, 0, 0);
    
    private int loopControl = 0;
    private SmartDashboardNumber rainbowControl = new SmartDashboardNumber("led/rainbow speed", 3);
    private SmartDashboardNumber larsonSpeed = new SmartDashboardNumber("led/larson speed", 1);
    
    private Superstructure superstructure = Superstructure.getInstance();

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
        if (loopControl % freq * 2 < freq) this.setLights(c);
        else this.setLights(OFF);
    }

    public void blinkSetLights(Color c, int blinks, int freq) {
        if (loopControl < freq * blinks) {
            if (loopControl % freq * 2 < freq) this.setLights(c);
            else this.setLights(OFF);
        }
        else setLights(c);
    }

    private int rainbowHue = 0;

    public void increaseHueControl() {rainbowControl.putNumber(rainbowControl.getNumber() + 1);}
    public void decreaseHueControl() {rainbowControl.putNumber(rainbowControl.getNumber() - 1);}

    public void rainbow() {
        for (var i = 0; i < buffer.getLength(); i++) {
          final var hue = (rainbowHue + (i * 180 / buffer.getLength())) % 180;
          buffer.setHSV(i, hue, 255, 128);
        }
        rainbowHue += rainbowControl.getNumber();
        rainbowHue %= 180;
    }

    private int larsonPosition = 0;
    private int larsonDirection = 1;
    private final int LARSON_SIZE = 15;

    public void larson(Color c) {
        larson(left, c);
        larson(back, c);
        larson(right, c);
    }

    public void larson(AddressableLEDBufferView view, Color c) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setLED(i, new Color(
                view.getLED(i).red * 0.5,
                view.getLED(i).green * 0.5,
                view.getLED(i).blue * 0.5
            ));
        }

        for (int offset = -(LARSON_SIZE / 2); offset <= LARSON_SIZE / 2; offset++) {
            int index = larsonPosition + offset;
            if (index < 0 || index >= view.getLength()) continue;

            double brightness = 1.0 - (Math.abs(offset) / (double)(LARSON_SIZE / 2 + 1));

            view.setLED(index, new Color(
                c.red * brightness,
                c.green * brightness,
                c.blue * brightness
            ));
        }

        if (loopControl % larsonSpeed.getNumber() == 0) {
            larsonPosition += larsonDirection;
            if (larsonPosition >= view.getLength() - 1) {
                larsonPosition = view.getLength() - 1;
                larsonDirection = -1;
            } else if (larsonPosition <= 0) {
                larsonPosition = 0;
                larsonDirection = 1;
            }
        }
    }
    
    @Override
    public void periodic() {
        loopControl++;

        switch (superstructure.getRobotState()) {
            case MANUAL_SHOT:
                this.blink(BLUE, 6);
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING:
                this.blink(GREEN, 6);
                break;
            case FULL_TRACKING:
                this.blink(RED, 6);
                break;
            case TURRET_TRACKING:
                this.larson(NOTE_ORANGE);
                break;
            case IDLE:
                this.larson(WHITE);
                break;
        }

        this.control.setData(buffer);
    }

    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}