// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Led;

public class PurpleLights extends CommandBase {
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private Led led;
  private boolean finish;

  /** Creates a new YellowLights. */
  public PurpleLights(Led led) {
    this.led = led;
    this.m_led = led.m_led;
    this.m_ledBuffer = led.m_ledBuffer;
    // this.m_led = m_led;
    // this.m_ledBuffer = m_ledBuffer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (var i = 0; i < Constants.LEDConstants.LED_LENGTH; i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 175, 35, 155);
    }

    m_led.setData(m_ledBuffer);
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
