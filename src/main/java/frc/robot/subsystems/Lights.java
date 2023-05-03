package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.util.VirtualSubsystem;

public class Lights extends VirtualSubsystem {
    private static Lights instance;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }
    
    // Robot state tracking
    public int loopCycleCount = 0;
    public boolean lowBattery = false;

    // LED IO
    private final AddressableLED leds;
    private final AddressableLEDBuffer buffer;

    // Constants
    private static final int length = 18;
    private static final int bottomLength = 7; // Placeholder value
    private static final int minLoopCycleCount = 10;
    private static final double shimmerExtremeness = 0.5;
    private static final double shimmerSpeed = 1;

    private Lights() {
        System.out.println("[Init] Creating LEDs");

        leds = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(buffer);
        leds.start();
    }

    public void periodic() {
        // if (lowBattery) {
        //     setBufferColor(Section.BOTTOM.start(), Section.BOTTOM.end(), Color.kOrangeRed);
        //     setBufferColor(Section.BOTTOM.start(), Section.BOTTOM.end(), ledColor);
        // }
        // else {
        //     setBufferColor(Section.FULL.start(), Section.FULL.end(), ledColor);
        // }

        // Exit during initial cycles
        loopCycleCount++;
        if (loopCycleCount < minLoopCycleCount) {
            return;
        }

        solid(Section.FULL, Color.kViolet);

        // Update LEDs
        leds.setData(buffer);
    }

    private void solid(Section section, Color color) {
        for (int i = section.start(); i < section.end(); i++) {
            buffer.setLED(i, color);
        }
    }
    
    private void shimmer(Section section, Color color) {
        for (int i = section.start(); i < section.end(); i++) {
            double brightnessFactor = shimmerExtremeness + Math.sin((loopCycleCount + i) * shimmerSpeed);
            buffer.setLED(i, new Color(color.red * brightnessFactor, color.green * brightnessFactor, color.blue * brightnessFactor));
        }
    }

    private void rainbow(Section section) {
        for (int i = section.start(); i < section.end(); i++) {
            int hue = ((loopCycleCount * 3) % 180 + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
    }

    // private void setBufferColor(int start, int end, Color color){
    //   for (var i = start; i < end; i++) {
    //     // Sets the specified LED to the RGB values for red
    //     buffer.setLED(i, color);
    //   }
    // }

    // private void setLowBattery(boolean lowBat) {
    //     lowBattery = lowBat;
    // }

    // private void disabled() {
    //     ledColor = Color.kViolet;
    // }

    // private void auto() {
    //     ledColor = Color.kOrange;
    // }

    // private void cube() {
    //     ledColor = Color.kPurple;
    // }

    // private void objectPicked() {
    //     ledColor = Color.kGreen;
    // }

    private static enum Section {
        UPPER,
        BOTTOM,
        FULL;

        private int start() {
            switch (this) {
                case UPPER:
                    return bottomLength;
                case BOTTOM:
                    return 0;
                case FULL:
                    return 0;
                default:
                    return 0;
          }
        }

        private int end() {
            switch (this) {
                case UPPER:
                    return length;
                case BOTTOM:
                    return bottomLength;
                case FULL:
                    return length;
                default:
                    return length;
            }
        }
    }

}
