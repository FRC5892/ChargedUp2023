// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ground_Intake;
import frc.robot.subsystems.LED;

public class intake extends CommandBase {
  private Ground_Intake ground_Intake;
  private boolean finish;

  /** Creates a new intake. */
  public intake(Ground_Intake intake) {
    ground_Intake = intake;
    finish = false;
    // this.ground_Intake = ground_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ground_Intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // delay(3.5);
    ground_Intake.closeClamp();
    // Timer.delay(1);
    // delay(3.5);
    ground_Intake.tiltUpward();
    // for (var i = 0;i < kicker_Buffer.getLength(); i++){

  }finish=true;

  // We can also use: new WaitCommand(5.0) if needed
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
