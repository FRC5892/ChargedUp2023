// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flapper_Intake extends SubsystemBase {
  /** Creates a new Flapper_Intake. */
  public DoubleSolenoid clampSolenoid;
  public DoubleSolenoid kickerSolenoid;
  public DoubleSolenoid tiltSolenoid;

  public Flapper_Intake() {
    clampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    kickerSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
    tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4,5);


  }
    public void openClamp(){
      clampSolenoid.set(Value.kForward);
    }
    public void closeClamp(){
      clampSolenoid.set(Value.kReverse);
    }
    public void openKicker(){
      kickerSolenoid.set(Value.kForward);
    }
    public void closeKicker(){
      kickerSolenoid.set(Value.kReverse);
    }
    public void opentilt(){
      tiltSolenoid.set(Value.kForward);
    }
    public void closetilt(){
      tiltSolenoid.set(Value.kReverse);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Clamp Piston", clampSolenoid);
    SmartDashboard.putData("Kicker Piston", kickerSolenoid);
    SmartDashboard.putData("Tilt Piston", tiltSolenoid);
  }
}
