package org.usfirst.frc.team5203.robot;

public class Constants {
	// Motor Controller IDS
	public final static int kFrontLeftTalonId = 1;
	public final static int kRearLeftTalonId = 2;
	public final static int kFrontRightTalonId = 3;
	public final static int kRearRightTalonId = 4;
	public final static int kIntakeTalonId = 10;
	
	// PID Values
	public final static double kIntakeP = 3;
	public final static double kIntakeI = 0;
	public final static double kIntakeD = 0;
	public final static double kDriveF = .2;
	
	public final static double wheelCircum = 18.84;
	
	// Speeds
	public final static double kIntakeSpeed = 800;
	public final static double kDriveSpeed = 100;
	public final static double kDriveAccel = 2;
	
	// Acceleration
	public final static double kIntakeAccelTime = .5;
	
}
