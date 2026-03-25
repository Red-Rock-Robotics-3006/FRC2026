package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import redrocklib.logging.SmartDashboardNumber;

public class LED extends SubsystemBase{

    private static LED instance = null;

    private AddressableLED control = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(77); // 118 for three strips

    // public AddressableLEDBufferView left = buffer.createView(79, 117);
    public AddressableLEDBufferView left = buffer.createView(76, 39);
    public AddressableLEDBufferView right = buffer.createView(0, 37);

    private Superstructure superstructure = Superstructure.getInstance();
    
    public final Color INIT_YELLOW = new Color(255, 165, 0);
    public final Color OFF = new Color(0, 0, 0);
    public final Color GREEN = new Color(0, 255, 0);
    public final Color MAGENTA = new Color(255, 0, 255);
    public final Color NOTE_ORANGE = new Color(255, 30, 0);
    public final Color WHITE = new Color(255, 255, 255);
    public final Color BLUE = new Color(0, 0, 255);
    public final Color RED = new Color(255, 0, 0);
    
    private int loopControl = 0;
    private SmartDashboardNumber rainbowControl = new SmartDashboardNumber("led/rainbow speed", 3);
    private SmartDashboardNumber policeSpeed = new SmartDashboardNumber("led/police speed", 6);

    private LED() {
        super("LED");

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
        int cycle = loopControl % (freq * 2);
        double brightness = cycle < freq
            ? (double) cycle / freq
            : (double)(freq * 2 - cycle) / freq;

        setLights(new Color(
            c.red * brightness,
            c.green * brightness,
            c.blue * brightness
        ));
    }

    public void blink(Color c1, Color c2, int freq) {
        int fullCycle = freq * 2;
        int segment = loopControl / fullCycle;
        int cycle = loopControl % fullCycle;

        double brightness = (cycle < freq)
            ? (double) cycle / freq
            : (double) (fullCycle - cycle) / freq;

        Color active = (segment % 2 == 0) ? c1 : c2;

        setLights(new Color(
            active.red * brightness,
            active.green * brightness,
            active.blue * brightness
        ));
    }

    public void blinkHard(Color c, int freq) {
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

    private final int LARSON_SIZE = 10;

    private static class LarsonState {
        int position = 0;
        int direction = 2;
    }

    private final LarsonState leftLarsonState  = new LarsonState();
    private final LarsonState rightLarsonState = new LarsonState();
    // private final LarsonState backLarsonState  = new LarsonState();

    public void larson(Color c, double speed) {
        larson(left,  leftLarsonState,  c, speed);
        larson(right, rightLarsonState, c, speed);
        // larson(back,  backLarsonState,  c);
        // this.buffer.setLED(0, OFF);
        this.buffer.setLED(38, OFF);
    }

    public void larson(AddressableLEDBufferView view, LarsonState state, Color c, double speed) {
        for (int i = 0; i < view.getLength(); i++) {
            view.setLED(i, new Color(
                view.getLED(i).red   * 0.5,
                view.getLED(i).green * 0.5,
                view.getLED(i).blue  * 0.5
            ));
        }

        for (int offset = -(LARSON_SIZE / 2); offset <= LARSON_SIZE / 2; offset++) {
            int index = state.position + offset;
            if (index < 0 || index >= view.getLength()) continue;

            double brightness = 1.0 - (Math.abs(offset) / (double)(LARSON_SIZE / 2 + 1));

            view.setLED(index, new Color(
                c.red   * brightness,
                c.green * brightness,
                c.blue  * brightness
            ));
        }

        if (loopControl % speed == 0) {
            state.position += state.direction;
            if (state.position >= view.getLength() - 1) {
                state.position   = view.getLength() - 1;
                state.direction  = -2;
            } else if (state.position <= 0) {
                state.position  = 0;
                state.direction = 2;
            }
        }
    }

    private boolean policeEnabled = false;

    private void police() {
        police(left);
        police(right);
        // police(back);
        // this.buffer.setLED(0, OFF);
        this.buffer.setLED(38, OFF);
    }

    private void police(AddressableLEDBufferView view) {
        int length = view.getLength();
        int halfLength = length / 2;
        int quarterLength = halfLength / 2;

        int state = (loopControl / (int) policeSpeed.getNumber()) % 8;

        for (int i = 0; i < length; i++) view.setLED(i, OFF);

        if (state == 0 || state == 2) {
            for (int i = 0; i < quarterLength; i++) view.setLED(i, RED);
            for (int i = halfLength; i < halfLength + quarterLength; i++) view.setLED(i, BLUE);
        }
        
        else if (state == 4 || state == 6) {
            for (int i = quarterLength; i < halfLength; i++) view.setLED(i, RED);
            for (int i = halfLength + quarterLength; i < length; i++) view.setLED(i, BLUE);
        }
    }

    private void togglePolice() {
        policeEnabled = !policeEnabled;
    }

    public Command togglePoliceCommand() {
        return Commands.runOnce(() -> this.togglePolice());
    }
    
    @Override
    public void periodic() {
        loopControl++;

        switch (superstructure.getRobotState()) {
            case MANUAL_SHOT:
                this.blink(BLUE, 5);
                break;
            case LERP_TUNING:
                this.blink(GREEN, 10);
                break;
            case SHOOTING_WHILE_MOVING:
                break;
            case SHOOTING_JAMMED:
                this.blink(NOTE_ORANGE, 5);
                break;
            case SHOOTING:
                this.blink(GREEN, 5);
                break;
            case FULL_TRACKING:
                this.setLights(RED);
                break;
            case TURRET_TRACKING:
                this.larson(WHITE, 1);
                break;
            case IDLE:
                // this.larson(WHITE, 5);
                this.rainbow();
                break;
        }

        if (policeEnabled) this.police();
        
        this.control.setData(buffer);
    }

    public static LED getInstance(){
        if (instance == null) instance = new LED();
        return instance;
    }
}