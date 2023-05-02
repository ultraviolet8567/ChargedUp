package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private static final int length = 5;

    private static Color ledColor = Color.kViolet;
    private static boolean lowBattery = false;

    public Lights() {
        // Must be a PWM header, not MXP or DIO
        leds = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        buffer = new AddressableLEDBuffer(length);
        leds.setLength(length);

    }

    public void periodic() {
        //Logger.getInstance().recordOutput("Lights/Color", "Red");

        // Set the data
        if(lowBattery){
          setBufferColor(Section.BOTTOM.start(), Section.BOTTOM.end(), Color.kOrangeRed);
          setBufferColor(Section.BOTTOM.start(), Section.BOTTOM.end(), ledColor);
        }else{
          setBufferColor(Section.FULL.start(), Section.FULL.end(), ledColor);
        }
        leds.setData(buffer);
        leds.start();

        //leds.setData(buffer);
        
        //Section section = Section.FULL;

        // for (int i = section.start(); i < section.end(); i++) {
        //     // Sets the specified LED to the RGB values for red
        //     m_ledBuffer.setLED(i, Color.kOrangeRed);;
        // }

        // m_led.setData(m_ledBuffer);
    }

    
    private static enum Section {
        BOTTOM,
        UPPER,
        FULL;

        private int start() {
          switch(this) {
              case BOTTOM:
                return 0;
              case UPPER:
              //this is a place holder value
                return 3;
              default:
                return 0;
          }
        }

        private int end() {
          switch(this) {
            case BOTTOM:
            //this is too
              return 2;
            case UPPER:
              return length - 1;
            default:
              return length - 1;
          }
        }
    }

    private void setBufferColor(int start, int end, Color color){
      for (var i = start; i < end; i++) {
        // Sets the specified LED to the RGB values for red
        buffer.setLED(i, color);
      }
    }

    public void setLowBattery(boolean lowBat){
      lowBattery = lowBat;
    }
    public void disabled(){
      ledColor = Color.kViolet;
    }
    public void auto(){
      //change this to rainbow
      ledColor = Color.kOrange;
    }
    public void cube(){
      ledColor = Color.kPurple;
    }
    public void objectPicked(){
      ledColor = Color.kGreen;
    }

}
