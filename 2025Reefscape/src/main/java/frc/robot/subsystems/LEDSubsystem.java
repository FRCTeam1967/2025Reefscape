// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 1;
    private static final int kLength = 768;
  
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    
    //private final LEDPattern m_rainbow;
  
    // private final Map<Double, Color> patternSteps;
    //private final LEDPattern whitePattern;
    //private final LEDPattern scrollingWhitePattern;
    // private final LEDPattern rainbowMask;
   //private final LEDPattern maskPatternBrightness;
    
    public LEDSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      
      //unmoving rainbow pattern
      //m_rainbow = LEDPattern.rainbow(255,255);

      //mask pattern w/ unmoving rainbow
      //maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      //patternSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      //whitePattern = LEDPattern.steps(patternSteps);
      //scrollingWhitePattern = whitePattern.scrollAtRelativeSpeed(Percent.per(Second).of(100));
    //   rainbowMask = m_rainbow.mask(scrollingWhitePattern);
      //maskPatternBrightness = rainbowMask.atBrightness(Percent.of(5));


      m_led.setLength(kLength);
      m_led.start();
      
      // Set the default command to turn the strip off, otherwise the last colors written by the last command to run will continue to be displayed.
      black();
    }
    
    //turns the led strip solid red
    public void red() {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for janksters red
            m_buffer.setRGB(i, 25, 5, 5);
        }
        m_led.setData(m_buffer);
    }
    //turns the led strip solid black
    public void black() {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_buffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_buffer);
    }

    // public void blue() {
    //   for (var i = 0; i < m_buffer.getLength(); i++) {
    //       // Sets the specified LED to the RGB values for red
    //       m_buffer.setRGB(i, 0, 0, 255);
    //   }
    //   m_led.setData(m_buffer);
    // }

    public void green() {
      for (var i = 0; i < m_buffer.getLength(); i++) {
          // Sets the specified LED to the RGB values for janksters red
          m_buffer.setRGB(i, 0, 25, 5);
      }
      m_led.setData(m_buffer);
  }
       
    // public void maskRainbow(){
    //    rainbowMask.applyTo(m_buffer);
    //    m_led.setData(m_buffer);
    // }

    //useless
    public void periodic() {
    
    }

}