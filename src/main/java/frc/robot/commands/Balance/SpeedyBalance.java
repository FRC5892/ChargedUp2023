// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Balance;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SpeedyBalance extends CommandBase {
  private Swerve s_Swerve;
  private Pigeon2 gyro;
  private boolean finish;
  private double previousAngle;
  private double currentAngle;

  private boolean foward;
  private double previousAngleSign;
  private double goUpDistance;
  private int snapBackDistance;

  /** Creates a new ActiceBalance. */
  public SpeedyBalance(Swerve swerve, Pigeon2 gyro) {
    this.s_Swerve = swerve;
    this.gyro = gyro;
    finish = false;
    foward = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousAngle = gyro.getRoll();
    previousAngleSign = Math.signum(previousAngle);
    snapBackDistance = 5;
    goUpDistance = 10;
    //immediately drive fast
    if(previousAngleSign == 1.0){
      s_Swerve.drive(new Translation2d(goUpDistance, 0).times(-Constants.Swerve.speedyBalanceSpeed),
    0, false, true);
      foward = true;
      
    }
    else{
      s_Swerve.drive(new Translation2d(goUpDistance, 0).times(Constants.Swerve.speedyBalanceSpeed),
    0, false, true);
    foward = false;
    }

    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    currentAngle = gyro.getRoll();
    double delta = previousAngle - currentAngle;
    if(foward){
      if(delta>previousAngle){
        finish=true;
      }
    else{
      if(delta<previousAngle){
        finish=true;
      }
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(foward){
    s_Swerve.drive(
          new Translation2d(snapBackDistance, 0).times(Constants.Swerve.speedyBackup),
          0, false, true);
    }
    else{
      s_Swerve.drive(
          new Translation2d(snapBackDistance, 0).times(-Constants.Swerve.speedyBackup),
          0, false, true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
