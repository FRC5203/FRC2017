package org.usfirst.frc.team5203.robot;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
	VCIDrive robotDrive;
	CANTalon frontLeft;
	CANTalon rearLeft;
	CANTalon frontRight;
	CANTalon rearRight;
	Timer timer;
	Spark climberMotor;
	Spark intakeMotor;
	Spark shooterMotor;
	double shooterSpeed = .8;
	
	final double intakeMultiplier = 1;
	// Channels for the wheels
	final int kFrontLeftChannel = 2;
	final int kRearLeftChannel = 3;
	final int kFrontRightChannel = 1;
	final int kRearRightChannel = 0;

	// The channel on the driver station that the joystick is connected to

	Joystick stick = new Joystick(0);
	public Robot() {
		frontLeft = new CANTalon(2);
		rearRight = new CANTalon(0);
		frontRight = new CANTalon(1);
		rearLeft = new CANTalon(3);
		climberMotor = new Spark(0);
		intakeMotor = new Spark(3);
		shooterMotor = new Spark(1);

		SmartDashboard.putNumber("ShooterSpeed", .8);
		
		robotDrive = new VCIDrive(frontLeft, rearLeft, frontRight, rearRight);
		robotDrive.setMaxRPM(500);
		
		robotDrive.m_rearLeftMotor.setVoltageRampRate(.2);
		robotDrive.m_rearRightMotor.setVoltageRampRate(.2);
		robotDrive.m_frontLeftMotor.setVoltageRampRate(.2);
		robotDrive.m_frontRightMotor.setVoltageRampRate(.2);
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); 
		robotDrive.setExpiration(0.1);
		SmartDashboard.putNumber("Set Max RPM", 500);
	}

	double time1;
	double currentGameTime;
	/**
	 * Runs the motors with Mecanum drive.
	 */
	@Override
	public void operatorControl() {
		robotDrive.setSafetyEnabled(true);
		
		time1 = timer.getFPGATimestamp();
		while (isOperatorControl() && isEnabled()) {
			SmartDashboard.putNumber("Max RPM", robotDrive.getMaxRPM());
			robotDrive.setMaxRPM(SmartDashboard.getNumber("Set Max RPM"));
			currentGameTime  = timer.getFPGATimestamp()-time1;
			SmartDashboard.putNumber("Timer Value", currentGameTime);
			
			double leftStickX = stick.getRawAxis(0);
			double leftStickY = stick.getRawAxis(1);
			double rightStickX = stick.getRawAxis(4);
			if(leftStickX > .1 && leftStickX < -.1){
				leftStickX = 0;
			}
			if(leftStickY > .1 && leftStickY < -.1){
				leftStickY = 0;
			}
			if(rightStickX > .1 && rightStickX < -.1){
				rightStickX = 0;
			}
			if(stick.getRawButton(1)){
				shooterMotor.set(SmartDashboard.getNumber("ShooterSpeed", .8));
			}
			else if(stick.getRawButton(5)){
				shooterMotor.set(-.6);
			}
			else{
				shooterMotor.set(0);
			}
			robotDrive.mecanumDrive_Cartesian(Math.pow(leftStickY,3), Math.pow(leftStickX,3), -1*Math.pow(rightStickX, 3), 0);

			//if(leftStickY > 0){
				intakeMotor.set(leftStickY*intakeMultiplier);
				SmartDashboard.putBoolean("Intake Enabled", true);
				SmartDashboard.putNumber("Intake Speed", leftStickY*intakeMultiplier);
			//}
			/*else{
				SmartDashboard.putBoolean("Intake Enabled", true);
				SmartDashboard.putNumber("Intake Speed", 0);
			}*/
			
			int pov = stick.getPOV();
			if(pov == 0){
				climberMotor.set(1);
				SmartDashboard.putBoolean("Climber Motor Enabled", true);
			}
			else if(pov == 180){
				climberMotor.set(-1);
				SmartDashboard.putBoolean("Climber Motor Enabled", true);
			}
			else{
				climberMotor.set(0);
				SmartDashboard.putBoolean("Climber Motor Enabled", false);
			}
			if(currentGameTime > 120 && pov != 0 && pov != 180 && SmartDashboard.getBoolean("Climber Enabled", true)){
				
				climberMotor.set(-.3);
			}
			
		
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}
}
