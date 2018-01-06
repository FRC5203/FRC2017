package org.usfirst.frc.team5203.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

public class Intake {
	
	private CANTalon intake_talon;
	
	public Intake(){
		intake_talon = new CANTalon(Constants.kIntakeTalonId);
		intake_talon.setF(0);
		intake_talon.changeControlMode(TalonControlMode.MotionMagic);
		intake_talon.setMotionMagicAcceleration(Constants.kIntakeSpeed/Constants.kIntakeAccelTime);
		intake_talon.setMotionMagicCruiseVelocity(Constants.kIntakeSpeed);
	}
	
	public void forward(){
		intake_talon.set(Constants.kIntakeSpeed);
	}
	
	public void backwards(){
		
	}
	
	public void stop(){
		
	}
	
}
