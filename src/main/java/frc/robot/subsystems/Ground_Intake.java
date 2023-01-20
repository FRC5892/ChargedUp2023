// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ground_Intake extends SubsystemBase {
  /** Creates a new Ground_Intake. */

  private CANSparkMax gIntakeMotor(int motorID, boolean inverted){
    CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushed);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
    return sparkMax;
  }

  private CANSparkMax leftMotor = gIntakeMotor(Constants.GROUND_INTAKE_PORTS[0], false);
  private CANSparkMax rightMotor = gIntakeMotor(Constants.GROUND_INTAKE_PORTS[0], true);
  private DoubleSolenoid actuationSolenoid;
  boolean actuated = false;
  public Ground_Intake() {
    actuationSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.GROUND_INTAKE_SOLENOID_PORTS[0], Constants.GROUND_INTAKE_SOLENOID_PORTS[1]);

  }

  public void driveArms(double leftSpeed, double rightSpeed) {
		leftMotor.set(leftSpeed);
		rightMotor.set(rightSpeed);
	}

	public void togglePistons(){
		if (actuated){
			actuationSolenoid.set(Value.kReverse);
			actuated = false;
		}
		else{
			actuationSolenoid.set(Value.kForward);
			actuated=true;
		}
	}

	public void pistonForward() {
		actuationSolenoid.set(Value.kForward);
	}

	public void pistonReverse() {
		actuationSolenoid.set(Value.kReverse);
	}

	public void stopMotors() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

  @Override
  public void periodic() {
    SmartDashboard.putData("Ground Intake Piston", actuationSolenoid);
  }
}
