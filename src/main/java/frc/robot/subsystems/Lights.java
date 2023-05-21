package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;

public class Lights extends VirtualSubsystem {
    private static Lights instance;

    public static Lights getInstance() {
        if (instance == null)
            instance = new Lights();
        return instance;
    }
    
    // Robot state tracking
    public int loopCycleCount = 0;
    public boolean lowBattery = false;
    public GamePiece gamePiece = GamePiece.CONE;
    public boolean pickUp = false;
    public RobotState state = RobotState.DISABLED;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    // Constants
    private static final int leftLength = 20;
    private static final int rightLength = 21;
    private static final int length = rightLength + leftLength;
    private static final int bottomLength = 7; // Placeholder value
    private static final int minLoopCycleCount = 10;
    private static final double shimmerExtremeness = 0.5;
    private static final double shimmerSpeed = 1;
    private static final double strobeTickSkip = 30;
    private static final double strobeTickDuration = 3;


    private Lights() {
        System.out.println("[Init] Creating LEDs");

        leds = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
    }

    public void periodic() {
        // Exit during initial cycles
        loopCycleCount++;
        if (loopCycleCount < minLoopCycleCount) {
            return;
        }
        
        // First branch off depending on what part of the match the robot is in

        // Disabled
        if (state == RobotState.DISABLED) {
            // Purple shimmer
        }

        // Autonomous
        else if (state == RobotState.AUTO) {
            // Rainbow
        }

        // Teleop
        else {
            // Game piece color
            
            // Pickup indicator
        }

        // Indicate low battery in every case
        
        // Update LEDs
        leds.setData(buffer);
    }

    private void solid(Section section, Color color) {
        if (section == Section.FULL) {
            solid(Section.LEFTFULL, color);
            solid(Section.RIGHTFULL, color);
        }
        else if (section == Section.BOTTOM) {
            solid(Section.LEFTBOTTOM, color);
            solid(Section.RIGHTBOTTOM, color);
        }
        else {
            for (int i = section.start(); i < section.end(); i++) {
                buffer.setLED(i, color);
            }
        }
    }
    
    private void shimmer(Section section, Color color) {
        if (section == Section.FULL) {
            shimmer(Section.LEFTFULL, color);
            shimmer(Section.RIGHTFULL, color);
        }
        else if (section == Section.BOTTOM) {
            shimmer(Section.LEFTBOTTOM, color);
            shimmer(Section.RIGHTBOTTOM, color);
        }
        else {
            for (int i = section.start(); i < section.end(); i++) {
                double brightnessFactor = shimmerExtremeness + Math.sin((loopCycleCount + i)*0.01) * shimmerSpeed;
                buffer.setLED(i, new Color(color.red * brightnessFactor, color.green * brightnessFactor, color.blue * brightnessFactor));
            }
        }
    }

    private void rainbow(Section section) {
        if (section == Section.FULL) {
            rainbow(Section.LEFTFULL);
            rainbow(Section.RIGHTFULL);
        }
        else if (section == Section.BOTTOM) {
            rainbow(Section.LEFTBOTTOM);
            rainbow(Section.RIGHTBOTTOM);
        }
        else {
            for (int i = section.start(); i < section.end(); i++) {
                int hue = ((loopCycleCount * 3) % 180 + (i * 180 / buffer.getLength())) % 180;
                buffer.setHSV(i, hue, 255, 128);
            }
        }
    }

    private void strobe(Section section, Color color){
        if (section == Section.FULL) {
            strobe(Section.LEFTFULL, color);
            strobe(Section.RIGHTFULL, color);
        }
        else if (section == Section.BOTTOM) {
            strobe(Section.LEFTBOTTOM, color);
            strobe(Section.RIGHTBOTTOM, color);
        }
        else {
            for (int i = section.start(); i < section.end(); i++) {
                if (loopCycleCount % (strobeTickSkip + 1) == strobeTickDuration) {
                    buffer.setLED(i, color);
                }
                else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }
    }

    private void strobeRainbow(Section section){
        if (loopCycleCount % (strobeTickSkip + 1) == strobeTickDuration) {
            rainbow(section);
        }
        else {
            solid(section, Color.kBlack);
        }
    }

    private static enum Section {
        FULL,
        BOTTOM,
        LEFTUPPER,
        LEFTBOTTOM,
        LEFTFULL,
        RIGHTUPPER,
        RIGHTBOTTOM,
        RIGHTFULL;

        private int start() {
            switch (this) {
                case FULL:
                    return 0;
                case LEFTUPPER:
                    return bottomLength;
                case LEFTBOTTOM:
                    return 0;
                case LEFTFULL:
                    return 0;
                case RIGHTUPPER:
                    return bottomLength + leftLength;
                case RIGHTBOTTOM:
                    return 0 + leftLength;
                case RIGHTFULL:
                    return 0 + leftLength;
                default:
                    return 0;
            }
        }

        private int end() {
            switch (this) {
                case FULL:
                    return length;
                case LEFTUPPER:
                    return leftLength;
                case LEFTBOTTOM:
                    return bottomLength;
                case LEFTFULL:
                    return leftLength;
                case RIGHTUPPER:
                    return length;
                case RIGHTBOTTOM:
                    return leftLength + bottomLength;
                case RIGHTFULL:
                    return length;
                default:
                    return 0;
            }
        }
    }
    
    public static enum GamePiece {
        CONE,
        CUBE;

        public String toString() {
            return this == GamePiece.CONE ? "Cone" : "Cube";
        }
    }

    public static enum RobotState {
        DISABLED,
        AUTO,
        TELEOP
    }
}
