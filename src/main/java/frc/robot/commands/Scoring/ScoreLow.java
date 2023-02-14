// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ScoreLow extends CommandBase {
  private Claw claw;
  private Arm arm;

  /** Creates a new ScoreLow. */
  public ScoreLow(Claw claw, Arm arm) {
    this.claw = claw;
    this.arm = arm;

    addRequirements(claw, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isClawDeployed = arm.returnClawPosition() == Value.kForward;

    if (!isClawDeployed) {
      arm.setClawPosition(Value.kForward);
    }

    claw.setMotors(Constants.ArmConstants.SPIT_OUT_SPEED);
    Timer.delay(5);
    claw.setMotors(0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
