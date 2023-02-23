// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PassiveBalance extends CommandBase {
  private Swerve m_Swerve;
  private boolean finish;

  /** Creates a new PassiveBalance. */
  public PassiveBalance(Swerve s_Swerve) {
    this.m_Swerve = s_Swerve;
    finish = false;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Swerve.setModule0(Constants.Swerve.Mod0.balanceOffset);
    m_Swerve.setModule1(Constants.Swerve.Mod1.balanceOffset);
    m_Swerve.setModule2(Constants.Swerve.Mod2.balanceOffset);
    m_Swerve.setModule3(Constants.Swerve.Mod3.balanceOffset);
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
