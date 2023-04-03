// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  private Pigeon2 gyro;

  /** Creates a new Led. */
  public Led(Pigeon2 gyro) {
    this.gyro = gyro;

    m_led = new AddressableLED(Constants.LEDConstants.LED_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.start();
    boolean balancing = gyro.getRoll() > 7 || gyro.getRoll() < -7;
    boolean balanced = gyro.getRoll() > -7 && gyro.getRoll() < 7;

    double changingBlueValue = Math.floor(Math.abs(gyro.getRoll() * 17));
    int finalBlue = (int) changingBlueValue;

    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // // orange
    // m_ledBuffer.setRGB(i, 255, 135, 0);
    // }

    // m_led.setData(m_ledBuffer);

    if (balancing) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // blue
        m_ledBuffer.setRGB(i, 0, 0, finalBlue);
      }

      m_led.setData(m_ledBuffer);
    }

    if (balanced) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // blue
        m_ledBuffer.setRGB(i, 255, 30, 0);
      }

      m_led.setData(m_ledBuffer);
    }

  }
}
