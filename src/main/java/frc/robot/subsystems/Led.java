// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Led extends SubsystemBase {
  public AddressableLED m_led;
  public AddressableLEDBuffer m_ledBuffer;
  private Pigeon2 gyro;
  double angle;
  DoubleSolenoid tiltpiston;
  Timer timer;
  Boolean on;

  /** Creates a new Led. */
  public Led(Pigeon2 gyro, DoubleSolenoid ts) {
    this.gyro = gyro;
    tiltpiston = ts;
    on = true;

    m_led = new AddressableLED(Constants.LEDConstants.LED_PORT);
    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.LED_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    timer = new Timer();
  }

  @Override
  public void periodic() {
    double roll = Math.abs(gyro.getRoll());
    double tilt = Math.abs(gyro.getPitch());

    if (roll > 1) {
      angle = roll;
    } else {
      angle = tilt;
    }
    // This method will be )called once per scheduler run
    m_led.start();
    boolean c1 = angle <= 1;
    boolean c2 = angle <= 1.5 && angle > 1;
    boolean c3 = angle <= 2 && angle > 1.5;
    boolean c4 = angle <= 2.5 && angle > 2;
    boolean c5 = angle <= 3 && angle > 2.5;
    boolean c6 = angle <= 3.5 && angle > 3;
    boolean c7 = angle <= 4 && angle > 3.5;
    boolean c8 = angle <= 4.5 && angle > 4;
    boolean c9 = angle <= 5 && angle > 4.5;
    boolean c10 = angle <= 5.5 && angle > 5;
    boolean c11 = angle <= 6 && angle > 5.5;
    boolean c12 = angle <= 6.5 && angle > 6;
    boolean c13 = angle <= 7 && angle > 6.5;
    boolean c14 = angle <= 7.5 && angle > 7;
    boolean c15 = angle <= 8 && angle > 7.5;
    boolean c16 = angle <= 8.5 && angle > 8;
    boolean c17 = angle <= 9 && angle > 8.5;
    boolean c18 = angle <= 9.5 && angle > 9;
    boolean c19 = angle <= 10 && angle > 9.5;
    boolean c20 = angle <= 10.5 && angle > 10;
    boolean c21 = angle <= 11 && angle > 10.5;
    boolean c22 = angle <= 11.5 && angle > 11;
    boolean c23 = angle <= 12 && angle > 11.5;
    boolean c24 = angle <= 12.5 && angle > 12;
    boolean c25 = angle <= 13 && angle > 12.5;
    boolean c26 = angle <= 13.5 && angle > 13;
    boolean c27 = angle <= 14 && angle > 13.5;
    boolean c28 = angle <= 14.5 && angle > 14;
    boolean c29 = angle <= 15 && angle > 14.5;
    boolean c30 = angle > 15;

    if (c1) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 30, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c2) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 42, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c3) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 52, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c4) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 60, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c5) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 68, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c6) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 75, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c7) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 81, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c8) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 87, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c9) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 93, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c10) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 98, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c11) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 104, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c12) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 109, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c13) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 114, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c14) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 119, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c15) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 123, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c16) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 128, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c17) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 133, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c18) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 137, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c19) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 141, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c20) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 146, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c21) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 150, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c22) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 154, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c23) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 158, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c24) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 162, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c25) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 166, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c26) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 170, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c27) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 174, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c28) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 178, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c29) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 182, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (c30) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 255, 186, 0);
      }
      m_led.setData(m_ledBuffer);
    }

    if (tiltpiston.get() == Value.kReverse) {
      timer.start();
      if (timer.hasElapsed(0.5)) {
        if (on) {
          m_led.stop();
          on = false;
          timer.reset();
        } else {
          m_led.start();
          on = true;
          timer.reset();
        }
      }
    } else {
      timer.stop();
      timer.reset();
      m_led.start();
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
