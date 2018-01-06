package org.usfirst.frc.team5203.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Timer;

public class PositionHandler {

	private ADXRS450_Gyro gyro;
	private BuiltInAccelerometer accel;
	private double posX = 0;
	private double posY = 0;
	private double angle = 0;
	private double velX;
	private double velY;
	private double alphaX = 0;
	private double alphaY = 0;
	public PositionHandler(ADXRS450_Gyro gyro, BuiltInAccelerometer accel){
		this.gyro = gyro;
		this.accel = accel;
	}
	
	public void update(){
		angle = gyro.getAngle();
		double accelerationX = accel.getX();
		double accelerationY = accel.getY();
		double posXPrev = posX;
		double posYPrev = posY;
		alphaX = accelerationX;
		alphaY = accelerationY;
		
	}
	
	public double getAlphaX(){
		update();
		return alphaX;
	}
	public double getAlphaY(){
		update();
		return alphaY;
	}
	public double getAngle(){
		update();
		return angle;
	}
	
}
