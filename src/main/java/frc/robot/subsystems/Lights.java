package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.LightConstants;

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

    private Lights() {
        System.out.println("[Init] Creating LEDs");

        leds = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(LightConstants.length);

        leds.setLength(LightConstants.length);
        leds.setData(buffer);
        leds.start();
    }

    public void periodic() {
        // Exit during initial cycles
        loopCycleCount++;
        if (loopCycleCount < LightConstants.minLoopCycleCount) {
            return;
        }
        
        // First branch off depending on what part of the match the robot is in
        
        // Disabled

        if (state == RobotState.DISABLED) {
            // Purple shimmer
            //solid(Section.FULL, Color.kViolet);
            rainbow(Section.FULL);
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
        lowBattery = (RobotController.getBatteryVoltage() < LightConstants.lowBatteryVoltage);
        lowBattery = true;
        //I don't know if it will let me change the bottom part if it's already been changed 
        if (lowBattery && loopCycleCount % LightConstants.lowBatteryFlashWait <= LightConstants.lowBatteryFlashWait - LightConstants.lowBatteryFlashDuration) {
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
                double brightnessFactor = LightConstants.shimmerExtremeness + Math.sin((loopCycleCount + i)*0.01) * LightConstants.shimmerSpeed;
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
                System.out.println(hue);
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
                if (loopCycleCount % (LightConstants.strobeTickSkip + 1) == LightConstants.strobeTickDuration) {
                    buffer.setLED(i, color);
                }
                else {
                    buffer.setHSV(i, 0, 0, 0);
                }
            }
        }
    }

    private void strobeRainbow(Section section){
        if (loopCycleCount % (LightConstants.strobeTickSkip + 1) == LightConstants.strobeTickDuration) {
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
        RIGHTFULL;

        private int start() {
            switch (this) {
                case FULL:
                    return 0;
                case LEFTUPPER:
                    return LightConstants.bottomLength;
                case LEFTBOTTOM:
                    return 0;
                case LEFTFULL:
                    return 0;
                case RIGHTUPPER:
                    return LightConstants.bottomLength + LightConstants.leftLength;
                case RIGHTBOTTOM:
                    return 1 + LightConstants.leftLength;
                case RIGHTFULL:
                    return 0 + LightConstants.leftLength;
                default:
                    return 0;
            }
        }

        private int end() {
            switch (this) {
                case FULL:
                    return LightConstants.length;
                case LEFTUPPER:
                    return LightConstants.leftLength;
                case LEFTBOTTOM:
                    return LightConstants.bottomLength;
                case LEFTFULL:
                    return LightConstants.leftLength;
                case RIGHTUPPER:
                    return LightConstants.length;
                case RIGHTBOTTOM:
                    return LightConstants.leftLength + LightConstants.bottomLength + 1;
                case RIGHTFULL:
                    return LightConstants.length;
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
