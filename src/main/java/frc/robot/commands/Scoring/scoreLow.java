// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ground_Intake;

public class scoreLow extends Command {
  private Ground_Intake ground_Intake;
  public static Timer timer;
  public static Timer timer2;

  // j
  /** Creates a new score. */
  public scoreLow(Ground_Intake intake) {
    this.ground_Intake = intake;
    timer = new Timer();
    timer2 = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ground_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer2.reset();
    timer2.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Tilt robot down, open clamp, send out kicker, wait 0.3s, bring kicker back in
    ground_Intake.tiltDownward();

    ground_Intake.openClamp();
    if (timer2.get() > 0.4) {
      timer.start();

      ground_Intake.returnKicker();

    }

  }

  // delay(1.5);
  // ground_Intake.returnKicker();

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer2.stop();
    ground_Intake.sendKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer2.get() > 0.4) {
      return (timer.get() > 0.05);
    } else {
      return false;
    }
  }
}
