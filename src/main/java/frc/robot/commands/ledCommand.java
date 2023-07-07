// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Ground_Intake;
//import frc.robot.subsystems.Led;
import frc.robot.subsystems.LedSub;


public class ledCommand extends CommandBase {
  //private Subsystem led;
  private Ground_Intake intake;
  private Pigeon2 gyro;
  double angle;
  boolean on;
  private LedSub sub;
  Timer timer;


  /** Creates a new ledCommand. */
  public ledCommand(Pigeon2 g, Ground_Intake i, LedSub l) {
    //this.led = l;
    this.gyro = g;
    this.intake = i;
    sub = l;
    on = true;
    timer = new Timer();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sub, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    double roll = Math.abs(gyro.getRoll());
    double tilt = Math.abs(gyro.getPitch());

    if (roll > 1) {
      angle = roll;
    } else {
      angle = tilt;
    }
    // This method will be )called once per scheduler run
    
    boolean c1 = angle <= 1;
    boolean c2 = angle <= 1.5 && angle > 1;
    boolean c3 = angle <= 2 && angle > 1.5;
    boolean c4 = angle <= 2.5 && angle > 2;
    boolean c5 = angle <= 3 && angle > 2.5;
    boolean c6 = angle <= 3.5 && angle > 3;
    boolean c7 = angle <= 4 && angle > 3.5;
    boolean c8 = angle <= 4.5 && angle > 4;
    boolean c9 = angle <= 5 && angle > 4.5;
    boolean c10 = angle <= 5.5 && angle > 5;
    boolean c11 = angle <= 6 && angle > 5.5;
    boolean c12 = angle <= 6.5 && angle > 6;
    boolean c13 = angle <= 7 && angle > 6.5;
    boolean c14 = angle <= 7.5 && angle > 7;
    boolean c15 = angle <= 8 && angle > 7.5;
    boolean c16 = angle <= 8.5 && angle > 8;
    boolean c17 = angle <= 9 && angle > 8.5;
    boolean c18 = angle <= 9.5 && angle > 9;
    boolean c19 = angle <= 10 && angle > 9.5;
    boolean c20 = angle <= 10.5 && angle > 10;
    boolean c21 = angle <= 11 && angle > 10.5;
    boolean c22 = angle <= 11.5 && angle > 11;
    boolean c23 = angle <= 12 && angle > 11.5;
    boolean c24 = angle <= 12.5 && angle > 12;
    boolean c25 = angle <= 13 && angle > 12.5;
    boolean c26 = angle <= 13.5 && angle > 13;
    boolean c27 = angle <= 14 && angle > 13.5;
    boolean c28 = angle <= 14.5 && angle > 14;
    boolean c29 = angle <= 15 && angle > 14.5;
    boolean c30 = angle > 15;

    if (c1) {
      for (var i = 0; i < sub.getLength(); i++){
      sub.setRGB(i, 255, 30, 0);
      }
      sub.setData();
    }

    if (c2) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 42, 0);
      }
      sub.setData();
    }

    if (c3) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 52, 0);
      }
      sub.setData();
    }

    if (c4) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 60, 0);
      }
      sub.setData();
    }

    if (c5) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 68, 0);
      }
      sub.setData();
    }

    if (c6) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 75, 0);
      }
      sub.setData();
    }

    if (c7) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 81, 0);
      }
      sub.setData();
    }

    if (c8) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 87, 0);
      }
      sub.setData();
    }

    if (c9) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 93, 0);
      }
      sub.setData();
    }

    if (c10) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 98, 0);
      }
      sub.setData();
    }

    if (c11) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 104, 0);
      }
      sub.setData();
    }

    if (c12) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 109, 0);
      }
      sub.setData();
    }

    if (c13) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 114, 0);
      }
      sub.setData();
    }

    if (c14) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 119, 0);
      }
      sub.setData();
    }

    if (c15) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 123, 0);
      }
      sub.setData();
    }

    if (c16) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 128, 0);
      }
      sub.setData();
    }

    if (c17) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 133, 0);
      }
      sub.setData();
    }

    if (c18) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 137, 0);
      }
      sub.setData();
    }

    if (c19) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 141, 0);
      }
      sub.setData();
    }

    if (c20) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 146, 0);
      }
      sub.setData();
    }

    if (c21) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 150, 0);
      }
      sub.setData();
    }

    if (c22) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 154, 0);
      }
      sub.setData();
    }

    if (c23) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 158, 0);
      }
      sub.setData();
    }

    if (c24) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 162, 0);
      }
      sub.setData();
    }

    if (c25) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 166, 0);
      }
      sub.setData();
    }

    if (c26) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 170, 0);
      }
      sub.setData();
    }

    if (c27) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 174, 0);
      }
      sub.setData();
    }

    if (c28) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 178, 0);
      }
      sub.setData();
    }

    if (c29) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 182, 0);
      }
      sub.setData();
    }

    if (c30) {
      for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, 186, 0);
      }
      sub.setData();
    }

    if (intake.getTilt() == Value.kReverse) {
      timer.start();
      if (timer.hasElapsed(0.5)) {
        if (on) {
          sub.stop();
          on = false;
          timer.reset();
        } else {
          sub.start();
          on = true;
          timer.reset();
        }
      }
    } else {
      timer.stop();
      timer.reset();
      sub.start();
    

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
