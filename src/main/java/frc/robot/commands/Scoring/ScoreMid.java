// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;

public class ScoreMid extends CommandBase {
  private Claw claw;
  private Arm arm;
  private boolean finish;

  private boolean stopping;

  /** Creates a new scoreGamePiece. */
  public ScoreMid(Claw intake, Arm arm) {
    this.claw = intake;
    this.arm = arm;
    stopping = false;
    finish = false;
    addRequirements(intake, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean armUnderMaxHeight = arm.returnEncoderValue() < Constants.ArmConstants.ARM_MAX_HEIGHT;
    boolean armAtMaxHeight = arm.returnEncoderValue() == Constants.ArmConstants.ARM_MAX_HEIGHT;

    if (armUnderMaxHeight) {
      arm.setArmUp();
      claw.setMotors(Constants.ArmConstants.WITH_GAMEPIECE_SPEED);
    }

    if (armAtMaxHeight) {
      arm.setExtendPistons(Value.kForward);
      arm.setClawPosition(Value.kForward);
      claw.outtakeGamePiece(-Constants.ArmConstants.SPIT_OUT_SPEED);
      stopping = true;
    }

    if (stopping) {
      arm.setClawPosition(Value.kReverse);
      arm.setExtendPistons(Value.kReverse);
      arm.setArmDown();
    }

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
