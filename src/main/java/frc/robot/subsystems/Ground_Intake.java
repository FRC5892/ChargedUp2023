// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Ground_Intake extends SubsystemBase {
	/* Solenoid */
	DoubleSolenoid clampSolenoid;
	DoubleSolenoid kickerSolenoid;
	DoubleSolenoid kickerSolenoid2;
	DoubleSolenoid tiltSolenoid;

	/** Creates a new Ground_Intake. */
	public Ground_Intake(DoubleSolenoid cs, DoubleSolenoid ks, DoubleSolenoid ks2, DoubleSolenoid ts) {
		clampSolenoid = cs;
		/*
		 * PORT[0] forward channel
		 * PORT[1] backward channel
		 */
		kickerSolenoid = ks;
		kickerSolenoid2 = ks2;
		/*
		 * PORT[2] forward channel
		 * PORT[3] backward channel
		 */
		this.tiltSolenoid = ts;
		/*
		 * PORT[4] forward channel
		 * PORT[5] backward channel
		 */
	}

	// open clamp
	public void openClamp() {
		clampSolenoid.set(Value.kForward);
	}

	// close clamp
	public void closeClamp() {
		clampSolenoid.set(Value.kReverse);
	}

	// push kicker out
	public void sendKicker() {
		kickerSolenoid.set(Value.kReverse);
		kickerSolenoid2.set(Value.kReverse);
	}

	// bring kicker back in
	public void returnKicker() {
		kickerSolenoid.set(Value.kForward);
		kickerSolenoid2.set(Value.kForward);
	}

	// Tilt robot forward
	public void tiltUpward() {
		tiltSolenoid.set(Value.kReverse);
	}

	// Tilt robot back upright
	public void tiltDownward() {
		tiltSolenoid.set(Value.kForward);
	}

	@Override
	public void periodic() {
		SmartDashboard.putData("Clamp Piston/s", clampSolenoid);
		SmartDashboard.putData("Kicker Piston", kickerSolenoid);
		SmartDashboard.putData("Tilt Piston", tiltSolenoid);
	}
}
