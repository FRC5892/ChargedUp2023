// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ground_Intake;

public class intake extends CommandBase {
  private Ground_Intake ground_Intake = new Ground_Intake();
  /** Creates a new intake. */
  public intake(Ground_Intake ground_Intake) {
    this.ground_Intake = ground_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ground_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ground_Intake.toggleClamp();
    "TODO: ADD TIMER!!"
    ground_Intake.toggleTilt();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
