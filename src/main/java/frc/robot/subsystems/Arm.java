// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private DoubleSolenoid positionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[0], Constants.ArmConstants.INTAKE_POSITION_SOLENOID_PORT[1]);
  private DoubleSolenoid extendSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ArmConstants.EXTEND_SOLENOID_PORT[0], Constants.ArmConstants.EXTEND_SOLENOID_PORT[1]);
  private CANSparkMax armMotor = new CANSparkMax(Constants.ArmConstants.ARM_MOTOR_PORT, MotorType.kBrushed);
  private VictorSP armEncoder = new VictorSP(Constants.ArmConstants.ARM_ENCODER);
  private Timer timer;
  /** Creates a new Arm. */
  public Arm() {
    positionSolenoid.set(Value.kReverse);
    extendSolenoid.set(Value.kReverse);
		timer = new Timer();
		timer.reset();
  }

  /* toggle pistons */
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

  /* open, close, set */  
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

  /* return values */
  public Value returnPositionPistons() {
    return positionSolenoid.get();
  }

  public Value returnExtendPistons() {
    return extendSolenoid.get();
  }

  public Double returnEncoderValue() {
    return armEncoder.get();
  }

  /*Mootroz */
  public void setArmMotor(double speed) {
		armMotor.set(speed);
	}

  public double getArmMotor(double speed) {
    return armMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
