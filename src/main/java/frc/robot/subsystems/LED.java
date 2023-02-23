// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED LEDs = new AddressableLED(0);
  private AddressableLEDBuffer LED_Buffer = new AddressableLEDBuffer(69);

  public LED() {
    LEDs.setLength(LED_Buffer.getLength());
    LEDs.setData(LED_Buffer);
    LEDs.start();
  }

  public void setColor(int r, int g, int b) {
    for (var i = 0; i < LED_Buffer.getLength(); i++) {
      LED_Buffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
