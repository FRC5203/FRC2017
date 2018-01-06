package org.usfirst.frc.team5203.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */
public class Robot extends SampleRobot {
	VCIDrive robotDrive; //Swap to RobotDrive to use ArcadeDrive for parade
	//RobotDrive robotDrive;
	CANTalon frontLeft;
	CANTalon rearLeft;
	CANTalon frontRight;
	CANTalon rearRight;
	Timer timer;
	SpeedController intakeMotor;
	SpeedController climberMotor;
	CANTalon shooterMotor;
	Spark popcorn;
	Spark popcorn2;
	ADXRS450_Gyro gyro;
	BuiltInAccelerometer accel;
	PositionHandler positionHandler;
	SendableChooser<String> autoChooser;
	double popcorn1Speed = .85;
	double popcorn2Speed = -.85;
	double shooterSpeed = 11;
	double intakeSpeed = 1;

	final double intakeMultiplier = 1;
	// Channels for the wheels
	final int kFrontLeftChannel = 1;
	final int kRearLeftChannel = 2;
	final int kFrontRightChannel = 3;
	final int kRearRightChannel = 4;

	// The channel on the driver station that the joystick is connected to

	Joystick stick = new Joystick(0);
	public Robot() {
		frontLeft = new CANTalon(Constants.kFrontLeftTalonId);
		rearLeft = new CANTalon(Constants.kRearLeftTalonId);
		frontRight = new CANTalon(Constants.kFrontRightTalonId);
		rearRight = new CANTalon(Constants.kRearRightTalonId);
		intakeMotor = new Spark(0);
		climberMotor = new CANTalon(10);
		shooterMotor = new CANTalon(15);
		shooterMotor.changeControlMode(TalonControlMode.Voltage);
		popcorn = new Spark(1);
		popcorn2 = new Spark(2);
		gyro = new ADXRS450_Gyro();
		accel = new BuiltInAccelerometer();
		positionHandler = new PositionHandler(gyro, accel);
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault("Gear Auto", "Gear");
		autoChooser.addObject("Gyro Auto", "Gyro");
		autoChooser.addObject("Ball Auto", "Ball");
		autoChooser.addObject("Center Gear", "Center");
		SmartDashboard.putData("Autonomous", autoChooser);

		CameraServer.getInstance().startAutomaticCapture();
		SmartDashboard.putBoolean("Drive Inverted", false);
		SmartDashboard.putNumber("ShooterSpeed", 11);
		SmartDashboard.putBoolean("Drive Enabled", true);
		SmartDashboard.putBoolean("Climber Enabled", true);
		SmartDashboard.putNumber("Popcorn Popper", .85);
		SmartDashboard.putNumber("Popcorn Popper 2", 0.85);
		SmartDashboard.putBoolean("Controls Swapped", false);
		robotDrive = new VCIDrive(frontLeft, rearLeft, frontRight, rearRight);
		//robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		robotDrive.setMaxRPM(500);

		robotDrive.m_rearLeftMotor.setVoltageRampRate(.2);
		robotDrive.m_rearRightMotor.setVoltageRampRate(.2);
		robotDrive.m_frontLeftMotor.setVoltageRampRate(.2);
		robotDrive.m_frontRightMotor.setVoltageRampRate(.2);
		 
		robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);
		robotDrive.setInvertedMotor(MotorType.kRearLeft, true); 
		robotDrive.setExpiration(0.5);
	}


	/*
	 * Runs the robot in autonomous
	 */
	public void autonomous(){
		gyro.reset();
		Timer.delay(.1);
		robotDrive.setVoltage();
		shooterMotor.setSafetyEnabled(false);
		robotDrive.setSafetyEnabled(false);
		String selectedAuto = autoChooser.getSelected();
		if(selectedAuto == "Gyro"){
			Alliance alliance = DriverStation.getInstance().getAlliance();
			if(alliance.name() == "Blue"){

			}
			if(alliance.name() == "Red"){

			}
			while(isAutonomous() && isEnabled()){

			}
		}
		if(selectedAuto == "Center"){
			robotDrive.setVoltage();
			time1 = Timer.getFPGATimestamp();
			double currentTime = 0;
			double turnPower = 0;
			while(isAutonomous() && isEnabled()){
				SmartDashboard.putNumber("Gyro Angle", positionHandler.getAngle());
				currentTime = Timer.getFPGATimestamp() - time1;
				if(currentTime < .75){
					turnPower = positionHandler.getAngle()/(Math.PI*2.5);
					SmartDashboard.putNumber("Turn Power", turnPower);
					SmartDashboard.putNumber("Accel X", positionHandler.getAlphaX());
					SmartDashboard.putNumber("Accel Y", positionHandler.getAlphaY());
					robotDrive.mecanumDrive_Cartesian(0.0, -.5, turnPower, positionHandler.getAngle());
				}
				else{
					robotDrive.mecanumDrive_Polar(0, 0, 0);
				}
				Timer.delay(.005);
			}
		}

		if(selectedAuto == "Gear"){
			int sideways = 90;
			double rotate = 0;
			Alliance alliance = DriverStation.getInstance().getAlliance();
			if(alliance.name() == "Blue"){
				sideways = 270;
				rotate = .09;
			}
			if(alliance.name() == "Red"){
				sideways = 90;
				rotate = -.09;
			}

			robotDrive.mecanumDrive_Polar(8, sideways, 0);
			shooterMotor.set(-SmartDashboard.getNumber("ShooterSpeed", 10.3)/2);
			Timer.delay(.2);
			robotDrive.mecanumDrive_Polar(0, 0, 0);
			Timer.delay(.1);
			popcorn.set(SmartDashboard.getNumber("Popcorn Popper", .85));
			popcorn2.set(SmartDashboard.getNumber("Popcorn Popper 2", .85));
			shooterMotor.set(SmartDashboard.getNumber("ShooterSpeed", 10.3));
			Timer.delay(6);
			shooterMotor.set(0);
			popcorn.set(0);
			popcorn2.set(0);
			robotDrive.mecanumDrive_Polar(8, sideways, 0);
			Timer.delay(.5);
			robotDrive.mecanumDrive_Polar(0, 0, 0);
			Timer.delay(.1);
			robotDrive.mecanumDrive_Polar(6, 0, rotate);
			Timer.delay(.9);
			robotDrive.mecanumDrive_Polar(0, 0, 0);
		}
		if(selectedAuto == "Ball"){
			int sideways = 90;
			double rotate = 0;
			Alliance alliance = DriverStation.getInstance().getAlliance();
			if(alliance.name() == "Blue"){
				sideways = 270;
				rotate = .2;
			}
			if(alliance.name() == "Red"){
				sideways = 90;
				rotate = -.2;
			}
			robotDrive.mecanumDrive_Polar(7, 0, 0);
			Timer.delay(.95);
			robotDrive.mecanumDrive_Polar(6, sideways, 0);
			Timer.delay(.9);
			robotDrive.mecanumDrive_Polar(0, 0, 0);
			intakeMotor.set(1);
			Timer.delay(1);
			robotDrive.mecanumDrive_Polar(8, -sideways, 0);
			Timer.delay(.2);
			robotDrive.mecanumDrive_Polar(2.5, 180, rotate);
			Timer.delay(1.05);
			robotDrive.mecanumDrive_Polar(0, 0, -.2);
			shooterMotor.set(-SmartDashboard.getNumber("ShooterSpeed", 10.3)/2);
			Timer.delay(.15);
			intakeMotor.set(0);
			robotDrive.mecanumDrive_Polar(3, 180, 0);
			Timer.delay(.1);
			robotDrive.mecanumDrive_Polar(0, 0, 0);
			Timer.delay(.1);
			popcorn.set(SmartDashboard.getNumber("Popcorn Popper", .85));
			popcorn2.set(SmartDashboard.getNumber("Popcorn Popper 2", .85));
			shooterMotor.set(SmartDashboard.getNumber("ShooterSpeed", 10.3));
			Timer.delay(10);
			popcorn.set(0);
			popcorn2.set(0);
			shooterMotor.set(0);


		}
	}


	double time1;
	double currentGameTime;
	/**
	 * Runs the motors with Mecanum drive.
	 */
	@Override
	public void operatorControl() {
		boolean driveToggled = false;
		boolean toggleButtonPressed = false;
		boolean toggleFirstLoop = false;
		//robotDrive.setPercent();
		robotDrive.setSafetyEnabled(true);
		shooterMotor.setSafetyEnabled(false);
		time1 = timer.getFPGATimestamp();
		popcorn1Speed = SmartDashboard.getNumber("Popcorn Popper", .85);
		popcorn2Speed = SmartDashboard.getNumber("Popcorn Popper 2", -.85);
		shooterSpeed = SmartDashboard.getNumber("ShooterSpeed", 11);

		while (isOperatorControl() && isEnabled()) {

			positionHandler.update();
			//SmartDashboard.putNumber("Accel X", positionHandler.getAlphaX());
			//SmartDashboard.putNumber("Accel Y", positionHandler.getAlphaY());
			//SmartDashboard.putNumber("Gyro Angle", positionHandler.getAngle());

			currentGameTime  = timer.getFPGATimestamp()-time1;
			double leftStickX = stick.getRawAxis(0);
			double leftStickY = stick.getRawAxis(1);
			double rightStickX = stick.getRawAxis(4);
			if(leftStickX < .05 && leftStickX > -.05){
				leftStickX = 0;
			}
			if(leftStickY < .05 && leftStickY > -.05){
				leftStickY = 0;
			}
			if(rightStickX < .05 && rightStickX > -.05){
				rightStickX = 0;
			}
			if(stick.getRawButton(1)){
				shooterMotor.set(shooterSpeed);
				popcorn.set(popcorn1Speed);
				popcorn2.set(popcorn2Speed);
			}
			else if(stick.getRawButton(5)){
				shooterMotor.set(shooterSpeed/-2);
			}
			else{
				shooterMotor.set(0);
				popcorn.set(0);
				popcorn2.set(0);
			}


			if(SmartDashboard.getBoolean("Drive Enabled", true)){
				if(SmartDashboard.getBoolean("Controls Swapped", false)){
					robotDrive.mecanumDrive_Cartesian(-.3*Math.pow(rightStickX,5), -Math.pow(leftStickY,5), -.5*Math.pow(leftStickX, 5), 0);
					//Swap control scheme for non-Mecanum parade driving
					//robotDrive.arcadeDrive(-leftStickX, -leftStickY*0.7);

				}else{
					robotDrive.mecanumDrive_Cartesian(-Math.pow(leftStickX,5), -Math.pow(leftStickY,5), -Math.pow(rightStickX, 5), 0);
					//robotDrive.arcadeDrive(-leftStickX, -leftStickY*0.7);
				}
			}


			if(leftStickY < 0){
				intakeMotor.set(leftStickY*intakeMultiplier);
			}
			else{
				intakeMotor.set(0);
			}
			if(stick.getRawButton(6)){
				intakeMotor.set(1);
			}else if(leftStickY < -.3){
				intakeMotor.set(-leftStickY);
			}
			else{
				intakeMotor.set(0);
			}

			int pov = stick.getPOV();
			if(stick.getRawAxis(3) > .3){
				climberMotor.set(stick.getRawAxis(3));
			}else if(currentGameTime > 120 && stick.getRawAxis(3) < .3){
				climberMotor.set(.35);
			}else{
				climberMotor.set(0);
			}

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}
	}



}
