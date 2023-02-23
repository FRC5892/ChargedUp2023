// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED kicker_LED = new AddressableLED(0);
  public AddressableLEDBuffer kicker_Buffer;

  public LED(AddressableLEDBuffer kicker_Buffer_) {
    this.kicker_Buffer = kicker_Buffer_;
    kicker_LED.setLength(kicker_Buffer.getLength());
    kicker_LED.setData(kicker_Buffer);
    kicker_LED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
