// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class ScoreMid extends CommandBase {
  private Intake intake;
  private Arm arm;

  boolean stopping;

  /** Creates a new scoreGamePiece. */
  public ScoreMid(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;
    stopping = false;
    addRequirements(intake, arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean armUnderMaxHeight = arm.returnEncoderValue() < Constants.ArmConstants.ARM_MAX_HEIGHT;
    boolean armAtMaxHeight = arm.returnEncoderValue() == Constants.ArmConstants.ARM_MAX_HEIGHT;
    
    if(armUnderMaxHeight) {
      //TODO pid
      arm.setArmUp();
      //TODO:test 
      intake.setMotors(Constants.ArmConstants.WITH_GAMEPIECE_SPEED);
    } 

    if (armAtMaxHeight) {
      arm.setExtendPistons(Value.kForward);
      arm.setClawPosition(Value.kForward);
      intake.setMotors(-Constants.ArmConstants.SPIT_OUT_SPEED);
      Timer.delay(5);
      intake.setMotors(0);
      stopping = true;
    }

    if (stopping) {
      arm.setClawPosition(Value.kReverse);
      arm.setExtendPistons(Value.kReverse);
      arm.setArmDown();
    }

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
