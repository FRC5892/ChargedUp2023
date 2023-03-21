// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
	private final AddressableLED m_leds;
	private final AddressableLEDBuffer m_buffer;

  /** Creates a new LEDSubsystem. */
 public LEDSubsystem() {
	m_leds = new AddressableLED(Constants.LEDConstants.LED_PORT);
    m_buffer = new AddressableLEDBuffer(1);
    m_leds.setData(m_buffer);
    m_leds.setLength(Constants.LEDConstants.LED_LENGTH);
    m_leds.start();
  }

  //set LED Orange (w/piece)
  public void setLEDOrange() {
		for (var i = 0; i < m_buffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			m_buffer.setRGB(i, 255, 85, 48);
		 }
		 
		 m_leds.setData(m_buffer);
	}

	//set LED White (w/o piece)
	public void setLEDWhite() {
		for (var i = 0; i < m_buffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			m_buffer.setRGB(i, 255, 255, 255);
		 }
		 
		 m_leds.setData(m_buffer);
	}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	m_leds.start();
  }
}
