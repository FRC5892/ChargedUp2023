// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


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
    //set subsystems
    this.gyro = g;
    this.intake = i;
    this.sub = l;

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
    //find direction and angle to use
    double roll = Math.abs(gyro.getRoll());
    double tilt = Math.abs(gyro.getPitch());

    if (roll > 1) {
      angle = roll;
    } else {
      angle = tilt;
    }
    //calculate blue value based on angle
    int point = (int)Math.floor((angle-0.1)*10)/5-1;
    if (point <= 0 ) {
        point = 0;
    } else if (point > 15) {
      point = 29;
    }
    final int[] blues = {30,42,52,60,68,75,81,87,93,98,104,109,114,119,123,128,133,137,141,146,150,154,158,162,166,170,174,178,182};
    //set color on all leds
    for (var i = 0; i < sub.getLength(); i++) {
        sub.setRGB(i, 255, blues[point], 0);
      }
      sub.setData();

    
      // i have no idea
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
