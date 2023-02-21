// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid positionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[0], Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[1]);
  private DoubleSolenoid extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      Constants.ArmConstants.EXTEND_SOLENOID_PORT[0], Constants.ArmConstants.EXTEND_SOLENOID_PORT[1]);

  /** Creates a new Arm. */
  public Pneumatics() {
    positionSolenoid.set(Value.kReverse);
    extendSolenoid.set(Value.kReverse);
  }

  /* toggle pistons */
  public void togglePositionPistons() {
    if (positionSolenoid.get() == Value.kForward) {
      positionSolenoid.set(Value.kReverse);
    }
  }

  public void toggleExtendPistons() {
    if (positionSolenoid.get() == Value.kForward) {
      positionSolenoid.set(Value.kReverse);
    }
  }

  /* open, close, set */
  public void releaseClawPiston() {
    positionSolenoid.set(Value.kForward);
  }

  public void retractClawPiston() {
    positionSolenoid.set(Value.kReverse);
  }

  public void openExtendPistons() {
    extendSolenoid.set(Value.kForward);
  }

  public void closeExtendPistons() {
    extendSolenoid.set(Value.kReverse);
  }

  public void setClawPosition(DoubleSolenoid.Value value) {
    positionSolenoid.set(value);
  }

  public void setExtendPistons(DoubleSolenoid.Value value) {
    extendSolenoid.set(value);
  }

  /* return values */
  public Value returnClawPosition() {
    return positionSolenoid.get();
  }

  public Value returnExtendPistonValue() {
    return extendSolenoid.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
