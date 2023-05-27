package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import edu.wpi.first.wpilibj.RobotController;

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
    public GamePiece gamePiece = GamePiece.REQCONE;
    public boolean pickUp = false;
    public static RobotState state = RobotState.DISABLED;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    private static final int leftLength = 20;
    private static final int rightLength = 21;
    private static final int smallLeftLength = 13;
    private static final int smallRightLength = 13;
    private static final int length = rightLength + leftLength + smallRightLength + smallLeftLength;
    private static final int bottomLength = 4; // Placeholder value
    private static final int minLoopCycleCount = 10;
    private static final double shimmerExtremeness = 0.5;
    private static final double shimmerSpeed = 1;
    private static final double strobeTickSkip = 30;
    private static final double strobeTickDuration = 3;
    private static final double lowBatteryVoltage = 10.0;
    private static final int lowBatteryFlashWait = 50;
    private static final int lowBatteryFlashDuration = 25;

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
        if (loopCycleCount <  minLoopCycleCount) {
            return;
        }
        
        // First branch off depending on what part of the match the robot is in
        
        // Disabled

        if (state == RobotState.DISABLED) {
            // Purple shimmer
            //solid(Section.FULL, Color.kViolet);
            solid(Section.FULL, Color.kDarkViolet);
        }

        // Autonomous
        else if (state == RobotState.AUTO) {
            // Rainbow
            rainbow(Section.FULL);
        }

        // Teleop
        else {
            // Game piece color
            
            // Pickup indicator
            solid(Section.FULL, gamePiece.color());
        }

        // Indicate low battery in every case
        lowBattery = (RobotController.getBatteryVoltage() <  lowBatteryVoltage);
        lowBattery = true;
        //I don't know if it will let me change the bottom part if it's already been changed 
        if (lowBattery && loopCycleCount % lowBatteryFlashWait <= lowBatteryFlashWait - lowBatteryFlashDuration) {
            solid(Section.BOTTOM, Color.kRed);
        }
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
                int hue = ((loopCycleCount * 3) % 180 + (i * 180 / leftLength)) % 180;

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
        UPPER,
        LEFTUPPER,
        LEFTBOTTOM,
        LEFTFULL,
        RIGHTUPPER,
        RIGHTBOTTOM,
        RIGHTFULL,
        LEFTSMALL,
        RIGHTSMALL;

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
                    return 1 + leftLength;
                case RIGHTFULL:
                    return 0 + leftLength;
                case RIGHTSMALL:
                    return 1 + length + smallLeftLength;
                case LEFTSMALL:
                    return 1 + length;
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
                    return leftLength + bottomLength + 1;
                case RIGHTFULL:
                    return length;
                case RIGHTSMALL:
                    return length + smallRightLength + smallLeftLength;
                case LEFTSMALL:
                    return length + smallLeftLength;
                default:
                    return 0;
            }
        }
    }
    
    public static enum GamePiece {
        REQCONE,
        REQCUBE;

        private Color color(){
            switch(this){
                case REQCONE: return Color.kYellow;
                case REQCUBE: return Color.kPurple;
                default: return Color.kYellow;
            }
        }

        public String toString() {
            switch(this){
                case REQCONE: return "Cone request";
                case REQCUBE: return "Cube request";
                default: return "Nothing Picked up or requested";
            }
        }
    }

    public static enum RobotState {
        DISABLED,
        AUTO,
        TELEOP;
    }
}
