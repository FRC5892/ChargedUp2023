package frc.robot.commands.Balance;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SpeedyBalance extends Command {
  private Swerve s_Swerve;
  private Pigeon2 gyro;
  private boolean finish;
  private Timer timer;

  private double previousAngle;
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
    previousAngle = gyro.getRoll().getValue();

    // immediately drive fast
    s_Swerve.drive(new Translation2d(1, 0).times(Constants.Swerve.speedyBalanceSpeed),
        0, false, true);

    finish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = gyro.getRoll().getValue();
    angleDiff = previousAngle - currentAngle;

    // how's the robot doin
    boolean robotTipped = angleDiff < 0;

    // how long to go backwards
    double snapBackTime = 0.8;
    boolean backedUpEnoughTime = timer.get() > snapBackTime;

    // drive while timer goes
    if (robotTipped) {
      timer.start();
      s_Swerve.drive(new Translation2d(1, 0).times(-Constants.Swerve.speedyBackup),
          0, false, true);
    }

    // stop once timer's done
    if (backedUpEnoughTime) {
      s_Swerve.drive(new Translation2d(0, 0).times(0),
          0, false, true);
      finish = true;
    }

    // preparing for next execute cycle if we're not tipped yet
    previousAngle = currentAngle;

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
