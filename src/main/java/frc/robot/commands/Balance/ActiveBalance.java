// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;;

public class ActiveBalance extends CommandBase {

  private Swerve s_Swerve;
  private Pigeon2 gyro;
  private boolean finish;

  /** Creates a new ActiceBalance. */
  public ActiveBalance(Swerve swerve, Pigeon2 gyro) {
    this.s_Swerve = swerve;
    this.gyro = gyro;
    finish = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gyroAngle = gyro.getRoll();

    if (gyroAngle > 10) {
      s_Swerve.drive(
          new Translation2d(1, 0).times(-Constants.Swerve.balanceSpeed),
          0, false, true);
    } else if (gyroAngle < -10) {
      s_Swerve.drive(
          new Translation2d(1, 0).times(Constants.Swerve.balanceSpeed),
          0, false, true);
    } else {
      s_Swerve.drive(
          new Translation2d(0, 0).times(Constants.Swerve.balanceSpeed),
          0, false, true);
      finish = true;
    }
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
