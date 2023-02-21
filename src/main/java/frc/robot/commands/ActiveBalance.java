// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class ActiveBalance extends CommandBase {

  private Swerve m_Swerve;

  private double error;
  private double currentAngle;
  private double drivePower;
  private Pigeon2 gyro;

  /**
   * Command to use Gyro data to resist the tip angle from the beam - to stabalize
   * and balanace
   */
  public ActiveBalance(Swerve s_Swerve, Pigeon2 gyro) {
    this.m_Swerve = s_Swerve;
    this.gyro = gyro;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a
    // controller joystick
    // Double currentAngle = -1 *
    // Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = gyro.getPitch();

    error = Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES - currentAngle;
    drivePower = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);

    // Limit the max power
    if (Math.abs(drivePower) > 0.4) {
      drivePower = Math.copySign(0.4, drivePower);
    }

    Translation2d translation = new Translation2d(drivePower, new Rotation2d(0));
    m_Swerve.drive(translation, 0, true, true);

    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new Translation2d(0, 0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the
                                                                             // specified threshold of being 'flat'
                                                                             // (gyroscope pitch of 0 degrees)
  }
}