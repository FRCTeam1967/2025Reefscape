package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {
    private static final int kPort = 0;
    private static final int kLength = 768;
  
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private final LEDPattern m_rainbow;
    private final LEDPattern m_scrollingRainbow;
    private final LEDPattern gradient;
    private final LEDPattern gradientBrightness;
    private final LEDPattern scrollingBrightness;
    private final LEDPattern patternBrightness;
    private final LEDPattern blinkPatternBrightness;
    

    private final LEDPattern base;
    private final LEDPattern pattern;
    private final LEDPattern blinkPattern;
    private final LEDPattern absolute;

    
    
    private static final Distance kLedSpacing = Meters.of(1 / 384.0);

    
    public LEDSubsystem() {
      m_led = new AddressableLED(kPort);
      m_buffer = new AddressableLEDBuffer(kLength);

      m_rainbow = LEDPattern.rainbow(255,128);
      m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);
      scrollingBrightness = m_scrollingRainbow.atBrightness(Percent.of(5));
      
      gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kMagenta);
      gradientBrightness = gradient.atBrightness(Percent.of(5));

      base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kChartreuse, Color.kDarkOrchid);
      pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(100));
      patternBrightness = pattern.atBrightness(Percent.of(5));
      absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), kLedSpacing);

      // 1.5 seconds on, 1.5 seconds off, for a total period of 3 seconds
      blinkPattern = base.blink(Seconds.of(2), Seconds.of(1));
      blinkPatternBrightness = blinkPattern.atBrightness(Percent.of(5));
    
    
      m_led.setLength(kLength);
      m_led.start();
      // Set the default command to turn the strip off, otherwise the last colors written by the last command to run will continue to be displayed.
      black();
    }
    
    public void red() {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for janksters red
            m_buffer.setRGB(i, 227, 15, 19);
        }
        m_led.setData(m_buffer);
    }
    public void black() {
        for (var i = 0; i < m_buffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_buffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_buffer);
    }
    
    public void scrollingRainbow() {
    // Update the buffer with the rainbow animation
        scrollingBrightness.applyTo(m_buffer);
    // Set the LEDs
        m_led.setData(m_buffer);
    }

    public void continuousGradient() {
        gradientBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer);

    }

    public void scrollDiscontinuousGradient() {
        patternBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer); 

    } 

    public void blinkGradient() {
        blinkPatternBrightness.applyTo(m_buffer);
        m_led.setData(m_buffer); 

    } 

    public void periodic() {
    }

}
