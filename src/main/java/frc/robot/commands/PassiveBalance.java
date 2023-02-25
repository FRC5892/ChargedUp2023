// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PassiveBalance extends CommandBase {
  private Swerve m_Swerve;
  private Timer timer;

  private boolean finish;

  /** Creates a new PassiveBalance. */
  public PassiveBalance(Swerve s_Swerve) {
    this.m_Swerve = s_Swerve;
    timer = new Timer();
    finish = false;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_Swerve.setModule0(Constants.Swerve.Mod0.balanceOffset);
    // m_Swerve.setModule1(Constants.Swerve.Mod1.balanceOffset);
    // m_Swerve.setModule2(Constants.Swerve.Mod2.balanceOffset);
    // m_Swerve.setModule3(Constants.Swerve.Mod3.balanceOffset);

    // double translationVal = translationLimiter
    //     .calculate(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    // double strafeVal = strafeLimiter
    //     .calculate(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    // double rotationVal = rotationLimiter
    //     .calculate(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));

    /* Drive */
    

    m_Swerve.drive(
      new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        1,
        false, true);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > 0.005);
  }
}
