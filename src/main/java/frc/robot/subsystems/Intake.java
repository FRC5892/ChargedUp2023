// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax leftMotor(int motorID, boolean inverted) {
    CANSparkMax leftMotor = new CANSparkMax(motorID, MotorType.kBrushed);
		leftMotor.restoreFactoryDefaults();
		leftMotor.setInverted(inverted);
		leftMotor.setIdleMode(IdleMode.kBrake);
    return leftMotor;
  }

  private CANSparkMax rightMotor(int motorID, boolean inverted) {
  CANSparkMax rightMotor = new CANSparkMax(motorID, MotorType.kBrushed);
  rightMotor.restoreFactoryDefaults();
  rightMotor.setInverted(inverted);
  rightMotor.setIdleMode(IdleMode.kBrake);
  return rightMotor;
  }

  private CANSparkMax leftMotor = leftMotor(Constants.ArmConstants.LEFT_INTAKE_MOTOR_PORT, false);
  private CANSparkMax rightMotor = rightMotor(Constants.ArmConstants.RIGHT_INTAKE_MOTOR_PORT, false);

  private MotorControllerGroup intakeMotors = new MotorControllerGroup(leftMotor, rightMotor);

  /** Creates a new Intake. */

  public Intake() {}

  public void setMotors(double speed) {
		intakeMotors.set(speed);
	}

  public double getMotors(double speed) {
    return leftMotor.get();
  }

	public void stopMotors() {
		intakeMotors.stopMotor();
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
