// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flapper_Intake;

public class Flapper_IntakeCommand extends CommandBase {
  /** Creates a new Flapper_IntakeCommand. */
  private Flapper_Intake flapper_Intake;
  public Flapper_IntakeCommand(Flapper_Intake fl) {
    // Use addRequirements() here to declare subsystem dependencies.
    flapper_Intake = fl;
    addRequirements(flapper_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flapper_Intake.openClamp();
    //flapper_Intake.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
