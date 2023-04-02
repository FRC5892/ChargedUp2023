package frc.robot.commands.Balance;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class SpeedyBalance extends CommandBase {
  private Swerve s_Swerve;
  private Pigeon2 gyro;
  private boolean finish;
  private Timer timer;
  private Timer timer2;

  private double initAngle;
  private double currentAngle;
  private double angleDiff;
  private double initSign;

  /** Creates a new ActiceBalance. */
  public SpeedyBalance(Swerve swerve, Pigeon2 gyro) {
    this.s_Swerve = swerve;
    this.gyro = gyro;
    timer2 = new Timer();

    finish = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer2.reset();
    // immediately drive fast

    finish = false;
    initAngle = gyro.getRoll();
    initSign = Math.signum(initAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentAngle = gyro.getRoll();

    double delta = currentAngle - initAngle;
    // previousAngle = currentAngle;
    SmartDashboard.putNumber("delta", delta);
    boolean robotTipped = false;
    if (initSign == 1.0) {
      robotTipped = delta < -2;
    } else {
      robotTipped = delta > 2;
    }
    // boolean backedUpEnough = timer2.get() > 0.5;
    int snapBackDistance = 10;

    // drive while timer goes
    if (robotTipped) {
      timer2.start();
      if (timer2.get() < 0.5) {
        s_Swerve.drive(new Translation2d(1, 0).times(initSign * Constants.Swerve.speedyBackup),
            0, false, true);
      } else {
        timer2.stop();
        finish = true;
      }
    } else {
      s_Swerve.drive(new Translation2d(1, 0).times(-initSign * 2),
          0, false, true);
    }

    // if (backedUpEnough) {
    // s_Swerve.drive(new Translation2d(0, 0).times(0),
    // 0, false, true);
    // finish = true;
    // }

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
