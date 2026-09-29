package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    int tick = 0;

    public LEDSubsystem() {
        m_led = new AddressableLED(5);
        
        m_ledBuffer = new AddressableLEDBuffer(150);
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
        
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        // Update the strip with the new buffer data
        m_led.setData(m_ledBuffer);
    }
    /**
    @Override
    public void periodic() {
        tick+=1;
        for (int i = 0; i<m_ledBuffer.getLength();i++){
            m_led.setData(i,i,i,i);
    }
    
    }
    */
}