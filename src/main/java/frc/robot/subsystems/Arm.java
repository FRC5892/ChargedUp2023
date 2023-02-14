// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
  private PIDController pid = new PIDController(Constants.ArmConstants.ARM_PIDF[0], Constants.ArmConstants.ARM_PIDF[1], Constants.ArmConstants.ARM_PIDF[2]);

  /** Creates a new Arm. */
  public Arm() {
    positionSolenoid.set(Value.kReverse);
    extendSolenoid.set(Value.kReverse);
    pid.setTolerance(Constants.ArmConstants.PID_POSITION_TOLERANCE);
		timer = new Timer();
		timer.reset();
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

  public Double returnEncoderValue() {
    return armEncoder.get();
  }

  /*Mootroz */
  public void setArmUp() {
		armMotor.set(pid.calculate(armEncoder.get(), Constants.ArmConstants.ARM_SETPOINT_UP));
	}

  public void setArmDown() {
		armMotor.set(pid.calculate(armEncoder.get(), Constants.ArmConstants.ARM_SETPOINT_DOWN));
	}

  public double getArmMotor() {
    pid.setTolerance(5, 10);
    pid.atSetpoint();
    return armMotor.get();
  }

  public boolean armAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
