// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ground_Intake;
import frc.robot.subsystems.LED;

public class retract extends CommandBase {
  private Ground_Intake ground_Intake;
  private Timer timer;
  private boolean finish;
  private LED LEDs;

  /** Creates a new score. */
  public retract(Ground_Intake intake, LED LEDs) {
    this.ground_Intake = intake;
    this.LEDs = LEDs;
    timer = new Timer();
    finish = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ground_Intake, LEDs);
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
    LEDs.setLEDWhite();
    finish = true;
  }

  // delay(1.5);
  // ground_Intake.returnKicker();

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
