// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ground_Intake extends SubsystemBase {
  /** Creates a new Ground_Intake. */



  private DoubleSolenoid clampSolenoid;
  private DoubleSolenoid kickerSolenoid;
  private DoubleSolenoid tiltSolenoid;
  
  public Ground_Intake() {
    clampSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
    Constants.GROUND_INTAKE_SOLENOID_PORTS[0], Constants.GROUND_INTAKE_SOLENOID_PORTS[1]);
	/* PORT[0] forward channel
	 * PORT[1] backward channel
	 */
	kickerSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
	Constants.GROUND_INTAKE_SOLENOID_PORTS[2], Constants.GROUND_INTAKE_SOLENOID_PORTS[3]);
	/* PORT[2] forward channel
	 * PORT[3] backward channel
	 */
	tiltSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
	Constants.GROUND_INTAKE_SOLENOID_PORTS[4], Constants.GROUND_INTAKE_SOLENOID_PORTS[5]);
	/* PORT[4] forward channel
	 * PORT[5] backward channel
	 */

  }


	public void toggleClamp(){
		if (clampSolenoid.get() == Value.kForward){
			clampSolenoid.set(Value.kReverse);
			
		}
		else{
			clampSolenoid.set(Value.kForward);
		}
	}

	public void toggleKicker(){
		if (kickerSolenoid.get() == Value.kForward){
			kickerSolenoid.set(Value.kReverse);
		}
		else{
			kickerSolenoid.set(Value.kForward);
		}
	}

	public void toggleTilt(){
		if (tiltSolenoid.get() == Value.kForward){
			tiltSolenoid.set(Value.kReverse);
		}
		else{
			tiltSolenoid.set(Value.kForward);
		}
	}


  @Override
  public void periodic() {
    SmartDashboard.putData("Clamp Piston/s", clampSolenoid);
	SmartDashboard.putData("Kicker Piston", kickerSolenoid);
	SmartDashboard.putData("Tilt Piston", tiltSolenoid);
  }
}
