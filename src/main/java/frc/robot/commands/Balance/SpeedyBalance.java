// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SpeedyBalance extends CommandBase {
  private Swerve s_Swerve;
  private Pigeon2 gyro;
  private boolean finish;
  private Timer timer;
  private Timer timer2;

  private double initialAngle;
  private double currentAngle;
  private double angleDiff;

  /** Creates a new ActiceBalance. */
  public SpeedyBalance(Swerve swerve, Pigeon2 gyro) {
    this.s_Swerve = swerve;
    this.gyro = gyro;
    timer = new Timer();
    finish = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    timer2.reset();
    initialAngle = gyro.getRoll();

    //immediately drive fast
    s_Swerve.drive(new Translation2d(1, 0).times(Constants.Swerve.speedyBalanceSpeed),
    0, false, true);

    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get the angle diff
    if (timer.get() == 0.05) {
      currentAngle = gyro.getRoll();
      angleDiff = initialAngle - currentAngle;
    }

    //reset timer & angles
    if (timer.get() == 0.06) {
      currentAngle = initialAngle;
      timer.reset();
    }
    
    //how's the robot doin
    boolean robotTipped = angleDiff < 0;
    boolean enoughBackingUp = timer2.get() > 0.5;
    
    //drive while timer goes
    if (robotTipped) {
      timer2.start();
      s_Swerve.drive(new Translation2d(1, 0).times(-Constants.Swerve.speedyBackup),
    0, false, true);
    }

    if (enoughBackingUp) {
      s_Swerve.drive(new Translation2d(0, 0).times(0),
    0, false, true);
      finish = true;
    }

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
