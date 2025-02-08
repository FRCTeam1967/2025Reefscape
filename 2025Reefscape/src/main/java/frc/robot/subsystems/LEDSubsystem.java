package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
    private final LEDPattern scrollRainbow;
    private final LEDPattern gradient;

    private final LEDPattern scrollContinuousGradient;
   
    
    private final LEDPattern base;
    private final LEDPattern pattern;
    private final LEDPattern blinkPattern;

    private final Map<Double, Color> patternSteps;
    private final LEDPattern whitePattern;
    private final LEDPattern scrollingWhitePattern;
    private final LEDPattern scrollingRedPattern;
    private final LEDPattern rainbowMask;
    private final LEDPattern black;
    private final LEDPattern red;

    private static final Distance kLedSpacing = Meters.of(1 / 384.0);

    /**creates new LED subsystem */
    public LEDSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);

      m_rainbow = LEDPattern.rainbow(255,128);
      scrollRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing).atBrightness(Percent.of(5));
      
      gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kMagenta);
      scrollContinuousGradient = gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing).atBrightness(Percent.of(5));

      base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kChartreuse, Color.kDarkOrchid);
      pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(100)).atBrightness(Percent.of(30));

      // 2 seconds on, 1 second off, for a total period of 3 seconds
      blinkPattern = base.blink(Seconds.of(2), Seconds.of(1)).atBrightness(Percent.of(5));
;
      patternSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
      whitePattern = LEDPattern.steps(patternSteps);
      scrollingWhitePattern = whitePattern.scrollAtRelativeSpeed(Percent.per(Second).of(100));
      rainbowMask = m_rainbow.mask(scrollingWhitePattern);

      black = LEDPattern.solid(Color.kBlack);
      red = LEDPattern.solid(Color.kRed);
      scrollingRedPattern = red.scrollAtRelativeSpeed(Percent.per(Second).of(100)).atBrightness(Percent.of(20));
    
      m_led.setLength(kLength);
      m_led.start();
    }
    
    /** Defines the red method */
    public void red() {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            /**Sets the specified LED to the RGB values for janksters red */
            m_buffer.setRGB(i, 227, 15, 19);
        }
        m_led.setData(m_buffer);
    }

    /** Defines the black method */
    public void blackPattern() {
        black.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void scrollingRainbow() {
   /**Set the buffer with the rainbow animation */
        scrollRainbow.applyTo(m_buffer);
    /** Set the LEDs*/
        m_led.setData(m_buffer);
    }

    public void scrollingContinuousGradient() {
        scrollContinuousGradient.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void scrollDiscontinuousGradient() {
        pattern.applyTo(m_buffer);
        m_led.setData(m_buffer); 
    }
    
    public void maskRainbow(){
        rainbowMask.applyTo(m_buffer);
        m_led.setData(m_buffer);
     }
 
    public void blinkGradient() {
        blinkPattern.applyTo(m_buffer);
        m_led.setData(m_buffer); 
    } 

    public void scrollingRed(){
        scrollingRedPattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void periodic() {
    }

}
