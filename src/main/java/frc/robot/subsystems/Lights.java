package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private static final int length = 60;

    public Lights() {
        System.out.println("sfd");
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(9);
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        Section section = Section.FULL;

        for (int i = section.start(); i < section.end(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setLED(i, Color.kOrangeRed);;
        }

        m_led.setData(m_ledBuffer);
    }
    
    private static enum Section {
        BOTTOM,
        BACK_UPPER,
        BACK,
        FRONT,
        FULL;

        private int start() {
            switch(this){
                case BOTTOM:
                    return 0;
                default:
                    return 0;
            }
        }

        private int end() {
            return length;
        }
    }
}
