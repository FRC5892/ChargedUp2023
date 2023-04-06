// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSub extends SubsystemBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  /** Creates a new LedSub. */
  public LedSub() {
    m_led = new AddressableLED(Constants.LEDConstants.LED_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


 
  public void setData(){
    m_led.setData(m_ledBuffer);
  }

  public void stop(){
    m_led.stop();
  }
  public void start(){
    m_led.start();
  }

  public double getLength(){
    return m_ledBuffer.getLength();
  }

  public void setRGB(int i, int a, int b, int c){
    m_ledBuffer.setRGB(i, a, b, c);
  }
}
