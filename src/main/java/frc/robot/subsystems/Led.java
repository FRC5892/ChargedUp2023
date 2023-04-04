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

    double angle = Math.abs(gyro.getRoll());
    // This method will be )called once per scheduler run
    m_led.start();
    boolean c1 = angle <= 1;
    boolean c2 = angle <= 2 && angle > 1;
    boolean c3 = angle <= 3 && angle > 2;
    boolean c4 = angle <= 4 && angle > 3;
    boolean c5 = angle <= 5 && angle > 4;
    boolean c6 = angle <= 6 && angle > 5;
    boolean c7 = angle <= 7 && angle > 6;
    boolean c8 = angle <= 8 && angle > 7;
    boolean c9 = angle <= 9 && angle > 8;
    boolean c10 = angle <= 10 && angle > 9;
    boolean c11 = angle <= 11 && angle > 10;
    boolean c12 = angle <= 12 && angle > 11;
    boolean c13 = angle <= 13 && angle > 12;
    boolean c14 = angle <= 14 && angle > 13;
    boolean c15 = angle > 14;

    if (c1) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 30, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c2) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 53, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c3) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 69, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c4) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 83, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c5) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 95, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c6) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 106, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c7) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 116, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c8) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 126, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c9) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 135, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c10) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 144, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c11) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 153, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c12) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 162, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c13) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 170, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c14) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 178, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c15) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 186, 0);
      }
      m_led.setData(m_ledBuffer);
    }
    // boolean balancing = gyro.getRoll() > 7 || gyro.getRoll() < -7;
    // boolean balanced = gyro.getRoll() > -7 && gyro.getRoll() < 7;

    // double changingBlueValue = Math.floor(Math.abs(gyro.getRoll() * 17));
    // int finalBlue = (int) changingBlueValue;

    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // // orange
    // m_ledBuffer.setRGB(i, 255, 135, 0);
    // }

    // m_led.setData(m_ledBuffer);

    // if (balancing) {
    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // // blue
    // m_ledBuffer.setRGB(i, 0, 0, finalBlue);
    // }

    // m_led.setData(m_ledBuffer);
    // }

    // if (balanced) {
    // for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    // // blue
    // m_ledBuffer.setRGB(i, 255, 30, 0);
    // }

    // m_led.setData(m_ledBuffer);
    // }

  }
}
