package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.*;

import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 0;
    private static final int kLength = 768;
  
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    
    private final LEDPattern m_rainbow;
    /* 
    private final LEDPattern m_scrollingRainbow;
    private final LEDPattern scrollingBrightness;

    private final LEDPattern gradient;
    private final LEDPattern gradientBrightness;
    
    private final LEDPattern patternBrightness;
    private final LEDPattern blinkPatternBrightness;
    
    private final LEDPattern base;
    private final LEDPattern pattern;
    private final LEDPattern blinkPattern;

    private final LEDPattern breathePattern;
    private final LEDPattern breathePatternBrightness;

    //private final Map<Double, Color> maskSteps;

    */
    private final Map<Double, Color> patternSteps;
    private final LEDPattern whitePattern;
    private final LEDPattern scrollingWhitePattern;
    private final LEDPattern rainbowMask;
   //private final LEDPattern maskPatternBrightness;
    
    private static final Distance kLedSpacing = Meters.of(1 / 384.0);

    
    public LEDSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);
      
      //unmoving rainbow pattern
      m_rainbow = LEDPattern.rainbow(255,255);

      /* 

      //moving rainbow pattern
      m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
      scrollingBrightness = m_scrollingRainbow.atBrightness(Percent.of(5));
      
      //unmoving continuous gradient 
      gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kMagenta);
      gradientBrightness = gradient.atBrightness(Percent.of(5));

      //moving gradient
      base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kChartreuse, Color.kDarkOrchid);
      pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(100));
      patternBrightness = pattern.atBrightness(Percent.of(5));

      // blink pattern (0.5 seconds on, 0.5 second off)
      blinkPattern = base.blink(Seconds.of(0.5), Seconds.of(0.5));
      blinkPatternBrightness = blinkPattern.atBrightness(Percent.of(5));
      
      //breathe pattern w/ unmoving rainbow
      breathePattern = m_rainbow.breathe(Seconds.of(1.5 ));
      breathePatternBrightness = breathePattern.atBrightness(Percent.of(5));

      */

      //mask pattern w/ unmoving rainbow
      //maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      patternSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      whitePattern = LEDPattern.steps(patternSteps);
      scrollingWhitePattern = whitePattern.scrollAtRelativeSpeed(Percent.per(Second).of(100));
      rainbowMask = m_rainbow.mask(scrollingWhitePattern);
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
            m_buffer.setRGB(i, 227, 15, 19);
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
    
    /*
    //turns the led strip into a scrolling rainbow
    public void scrollingRainbow() {
    // Update the buffer with the rainbow animation
        scrollingBrightness.applyTo(m_buffer);
    // Set the LEDs
        m_led.setData(m_buffer);
    }

    
    //turns the led strip into an unmoving continuous gradient
    public void continuousGradient() {
        gradientBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer);

    }
    
    
    //moving discontinuous gradient
    public void scrollDiscontinuousGradient() {
        patternBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer); 

    } 
    
    //turns the led strip into a blinking gradient
    public void blinkGradient() {
        blinkPatternBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer); 
        
    } 
     
    public void breatheGradient(){
       breathePatternBrightness.applyTo(m_buffer);
       m_led.setData(m_buffer);

    }

    */
   
    public void maskRainbow(){
       rainbowMask.applyTo(m_buffer);
       m_led.setData(m_buffer);
    }

    
    //useless
    public void periodic() {
    
    }

}
