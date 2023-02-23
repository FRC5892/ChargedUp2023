// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ground_Intake;
import frc.robot.subsystems.LED;

public class retract extends CommandBase {
  private Ground_Intake ground_Intake;
  private double d;
  private Timer timer;
  private boolean finish;
  private LED kicker_Buffer;
  AddressableLEDBuffer LEDs;

  /** Creates a new score. */
  public retract(Ground_Intake intake, LED kicker_Buffer_) {
    this.ground_Intake = intake;
    kicker_Buffer = kicker_Buffer_;
    timer = new Timer();
    finish = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ground_Intake, kicker_Buffer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ground_Intake.sendKicker();
    for (var i = 0; i < LEDs.getLength(); i++) {
      LEDs.setRGB(i, 255, 126, 0);
    }
    finish = true;
  }

  // delay(1.5);
  // ground_Intake.returnKicker();

  private void delay(double d) {
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
