// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  public LED() {
  }

  // set LED Orange (w/piece)
  public void setLEDOrange() {
    for (var i = 0; i < RobotContainer.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      RobotContainer.m_ledBuffer.setRGB(i, 255, 85, 48);
    }

    RobotContainer.m_led.setData(RobotContainer.m_ledBuffer);
  }

  // set LED White (w/o piece)
  public void setLEDWhite() {
    for (var i = 0; i < RobotContainer.m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      RobotContainer.m_ledBuffer.setRGB(i, 255, 255, 255);
    }

    RobotContainer.m_led.setData(RobotContainer.m_ledBuffer);
  }
}
