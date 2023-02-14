// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class RunIntakeRollars extends CommandBase {
  private Intake intake;
  private Arm arm;
  private boolean finished;
  
  /** Creates a new RunIntakeRollars. */
  public RunIntakeRollars(Intake intake, Arm arm) {
    this.intake = intake;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.finished = false;
    boolean isTriggerActive = RobotContainer.driver.getRightTriggerAxis() > 0.05;
    boolean isClawDeployed = arm.returnClawPosition() == Value.kForward;

    if (!isClawDeployed && isTriggerActive) {
      arm.setClawPosition(Value.kForward);
    }

    if (isTriggerActive) {
      intake.setMotors(Constants.ArmConstants.INTAKE_SPEED);
    }else {
      intake.setMotors(0);
    }
    
    this.finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.finished;
  }
}
