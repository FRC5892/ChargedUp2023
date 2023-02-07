// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private DoubleSolenoid positionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[0], Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[1]);
  private DoubleSolenoid extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ArmConstants.EXTEND_SOLENOID_PORT[0], Constants.ArmConstants.EXTEND_SOLENOID_PORT[1]);
  private Timer timer;
  /** Creates a new Arm. */
  public Arm() {
    positionSolenoid.set(Value.kReverse);
    extendSolenoid.set(Value.kReverse);
		timer = new Timer();
		timer.reset();
  }

  public void togglePositionPistons() {
    if (positionSolenoid.get() == Value.kForward) {
      positionSolenoid.set(Value.kReverse);
    } else {
      positionSolenoid.set(Value.kForward);
    }
  }

  public void toggleExtendPistons() {
    if (positionSolenoid.get() == Value.kForward) {
      positionSolenoid.set(Value.kReverse);
    } else {
      positionSolenoid.set(Value.kForward);
    }
  }

  public void openPositionPistons() {
    positionSolenoid.set(Value.kForward);
  }

  public void closePositionPistons() {
    positionSolenoid.set(Value.kReverse);
  }

  public void openExtendPistons() {
    extendSolenoid.set(Value.kForward);
  }

  public void closeExtendPistons() {
    extendSolenoid.set(Value.kReverse);
  }

  public void setPositionPistons(DoubleSolenoid.Value value) {
		positionSolenoid.set(value);
	}

  public void setExtendPistons(DoubleSolenoid.Value value) {
		extendSolenoid.set(value);
	}

  public Value returnPositionPistons() {
    return positionSolenoid.get();
  }

  public Value returnExtendPistons() {
    return extendSolenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
