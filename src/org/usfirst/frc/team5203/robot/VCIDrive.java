package org.usfirst.frc.team5203.robot;

import static java.util.Objects.requireNonNull;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tInstances;
import edu.wpi.first.wpilibj.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.hal.HAL;

public class VCIDrive implements MotorSafety{
	protected MotorSafetyHelper m_safetyHelper;

	  /**
	   * The location of a motor on the robot for the purpose of driving.
	   */
	  public enum MotorType {
	    kFrontLeft(0), kFrontRight(1), kRearLeft(2), kRearRight(3);

	    @SuppressWarnings("MemberName")
	    public final int value;

	    private MotorType(int value) {
	      this.value = value;
	    }
	  }

	  public static final double kDefaultExpirationTime = 0.1;
	  public static final double kDefaultSensitivity = 0.5;
	  public static final double kDefaultMaxOutput = 14;
	  protected static final int kMaxNumberOfMotors = 4;
	  protected double m_sensitivity;
	  protected double m_maxOutput;
	  private double maxRPM = 500;
	  public double getMaxRPM() {
		return maxRPM;
	}

	public void setMaxRPM(double maxRPM) {
		this.maxRPM = maxRPM;
	}
	private double kP = 1;
	private double kI = 0;
	private double kD = 0;
	public CANTalon m_frontLeftMotor;
	public CANTalon m_frontRightMotor;
	public CANTalon m_rearLeftMotor;
	public CANTalon m_rearRightMotor;
  protected boolean m_allocatedSpeedControllers;
  protected static boolean kArcadeRatioCurve_Reported = false;
  protected static boolean kTank_Reported = false;
  protected static boolean kArcadeStandard_Reported = false;
  protected static boolean kMecanumCartesian_Reported = false;
  protected static boolean kMecanumPolar_Reported = false;

	  /**
	   * Constructor for VCIDrive with 2 motors specified with channel numbers. Set up parameters for
	   * a two wheel drive system where the left and right motor pwm channels are specified in the call.
	   * This call assumes Talons for controlling the motors.
	   *
	   * @param leftMotorChannel  The PWM channel number that drives the left motor.
	   * @param rightMotorChannel The PWM channel number that drives the right motor.
	   */
	  public VCIDrive(final int leftMotorChannel, final int rightMotorChannel) {
	    m_sensitivity = kDefaultSensitivity;
	    m_maxOutput = kDefaultMaxOutput;
	    m_frontLeftMotor = null;
	    m_rearLeftMotor = new CANTalon(leftMotorChannel);
	    m_frontRightMotor = null;
	    m_rearRightMotor = new CANTalon(rightMotorChannel);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_allocatedSpeedControllers = true;
	    setupMotorSafety();
	    mecanumDrive_Cartesian(0, 0, 0, 0);
	  }

	  /**
	   * Constructor for VCIDrive with 4 motors specified with channel numbers. Set up parameters for
	   * a four wheel drive system where all four motor pwm channels are specified in the call. This
	   * call assumes Talons for controlling the motors.
	   *
	   * @param frontLeftMotor  Front left motor channel number
	   * @param rearLeftMotor   Rear Left motor channel number
	   * @param frontRightMotor Front right motor channel number
	   * @param rearRightMotor  Rear Right motor channel number
	   */
	  public VCIDrive(final int frontLeftMotor, final int rearLeftMotor, final int frontRightMotor,
	                    final int rearRightMotor) {
	    m_sensitivity = kDefaultSensitivity;
	    m_maxOutput = kDefaultMaxOutput;
	    m_rearLeftMotor = new CANTalon(rearLeftMotor);
	    m_rearRightMotor = new CANTalon(rearRightMotor);
	    m_frontLeftMotor = new CANTalon(frontLeftMotor);
	    m_frontRightMotor = new CANTalon(frontRightMotor);
	    m_frontLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_rearRightMotor.changeControlMode(TalonControlMode.Speed);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_allocatedSpeedControllers = true;
	    setupMotorSafety();
	    mecanumDrive_Cartesian(0, 0, 0, 0);
	  }

	  /**
	   * Constructor for VCIDrive with 2 motors specified as SpeedController objects. The
	   * SpeedController version of the constructor enables programs to use the VCIDrive classes with
	   * subclasses of the SpeedController objects, for example, versions with ramping or reshaping of
	   * the curve to suit motor bias or dead-band elimination.
	   *
	   * @param leftMotor  The left SpeedController object used to drive the robot.
	   * @param rightMotor the right SpeedController object used to drive the robot.
	   */
	  public VCIDrive(CANTalon leftMotor, CANTalon rightMotor) {
	    if (leftMotor == null || rightMotor == null) {
	      m_rearLeftMotor = m_rearRightMotor = null;
	      throw new NullPointerException("Null motor provided");
	    }
	    m_frontLeftMotor = null;
	    m_rearLeftMotor = leftMotor;
	    m_frontRightMotor = null;
	    m_rearRightMotor = rightMotor;
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Speed);
	    m_sensitivity = kDefaultSensitivity;
	    m_maxOutput = kDefaultMaxOutput;
	    m_allocatedSpeedControllers = false;
	    setupMotorSafety();
	    mecanumDrive_Cartesian(0, 0, 0, 0);
	  }

	  /**
	   * Constructor for VCIDrive with 4 motors specified as SpeedController objects. Speed controller
	   * input version of VCIDrive (see previous comments).
	   *
	   * @param rearLeftMotor   The back left SpeedController object used to drive the robot.
	   * @param frontLeftMotor  The front left SpeedController object used to drive the robot
	   * @param rearRightMotor  The back right SpeedController object used to drive the robot.
	   * @param frontRightMotor The front right SpeedController object used to drive the robot.
	   */
	  public VCIDrive(CANTalon frontLeftMotor, CANTalon rearLeftMotor,
			  CANTalon frontRightMotor, CANTalon rearRightMotor) {
	    m_frontLeftMotor = requireNonNull(frontLeftMotor, "frontLeftMotor cannot be null");
	    m_rearLeftMotor = requireNonNull(rearLeftMotor, "rearLeftMotor cannot be null");
	    m_frontRightMotor = requireNonNull(frontRightMotor, "frontRightMotor cannot be null");
	    m_rearRightMotor = requireNonNull(rearRightMotor, "rearRightMotor cannot be null");
	    m_frontRightMotor.changeControlMode(TalonControlMode.PercentVbus);
	    m_frontLeftMotor.changeControlMode(TalonControlMode.PercentVbus);
	    m_rearRightMotor.changeControlMode(TalonControlMode.PercentVbus);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.PercentVbus);
	    m_frontRightMotor.setPID(kP,kI,kD);
	    m_frontRightMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    m_frontLeftMotor.setPID(kP,kI,kD);
	    m_frontLeftMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    m_rearRightMotor.setPID(kP,kI,kD);
	    m_rearRightMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    m_rearLeftMotor.setPID(kP,kI,kD);
	    m_rearLeftMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
	    m_sensitivity = kDefaultSensitivity;
	    m_maxOutput = kDefaultMaxOutput;
	    m_allocatedSpeedControllers = false;
	    setupMotorSafety();
	    mecanumDrive_Cartesian(0, 0, 0, 0);
	  }

	  double multiplier = 1;
	  
	public void setVoltage(){  

	 	multiplier = 1;
	    m_frontRightMotor.changeControlMode(TalonControlMode.Voltage);
	    m_frontLeftMotor.changeControlMode(TalonControlMode.Voltage);
	    m_rearRightMotor.changeControlMode(TalonControlMode.Voltage);
	    m_rearLeftMotor.changeControlMode(TalonControlMode.Voltage);
	    m_frontRightMotor.setCurrentLimit(30);
	    m_frontLeftMotor.setCurrentLimit(30);
	    m_rearRightMotor.setCurrentLimit(30);
	    m_rearLeftMotor.setCurrentLimit(30);
	    m_frontRightMotor.EnableCurrentLimit(true);
	    m_frontLeftMotor.EnableCurrentLimit(true);
	    m_rearRightMotor.EnableCurrentLimit(true);
	    m_rearLeftMotor.EnableCurrentLimit(true);
	}
	  
	  public void setPercent(){
		  	multiplier = 1;
		    m_frontRightMotor.changeControlMode(TalonControlMode.PercentVbus);
		    m_frontLeftMotor.changeControlMode(TalonControlMode.PercentVbus);
		    m_rearRightMotor.changeControlMode(TalonControlMode.PercentVbus);
		    m_rearLeftMotor.changeControlMode(TalonControlMode.PercentVbus);
		    m_frontRightMotor.setCurrentLimit(30);
		    m_frontLeftMotor.setCurrentLimit(30);
		    m_rearRightMotor.setCurrentLimit(30);
		    m_rearLeftMotor.setCurrentLimit(30);
		    m_frontRightMotor.EnableCurrentLimit(true);
		    m_frontLeftMotor.EnableCurrentLimit(true);
		    m_rearRightMotor.EnableCurrentLimit(true);
		    m_rearLeftMotor.EnableCurrentLimit(true);
	  }
	  

	  /**
	   * Drive method for Mecanum wheeled robots.
	   *
	   * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
	   * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
	   * top, the roller axles should form an X across the robot.
	   *
	   * <p>This is designed to be directly driven by joystick axes.
	   *
	   * @param x         The speed that the robot should drive in the X direction. [-1.0..1.0]
	   * @param y         The speed that the robot should drive in the Y direction. This input is
	   *                  inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
	   * @param rotation  The rate of rotation for the robot that is completely independent of the
	   *                  translation. [-1.0..1.0]
	   * @param gyroAngle The current angle reading from the gyro. Use this to implement field-oriented
	   *                  controls.
	   */
	  @SuppressWarnings("ParameterName")
	  public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
	    if (!kMecanumCartesian_Reported) {
	      HAL.report(tResourceType.kResourceType_RobotDrive, getNumMotors(),
	          tInstances.kRobotDrive_MecanumCartesian);
	      kMecanumCartesian_Reported = true;
	    }
	    @SuppressWarnings("LocalVariableName")
	    double xIn = x;
	    @SuppressWarnings("LocalVariableName")
	    double yIn = y;
	    // Negate y for the joystick.
	    yIn = -yIn;
	    // Compenstate for gyro angle.
	    double[] rotated = rotateVector(xIn, yIn, gyroAngle);
	    xIn = rotated[0];
	    yIn = rotated[1];

	    double[] wheelSpeeds = new double[kMaxNumberOfMotors];
	    wheelSpeeds[MotorType.kFrontLeft.value] = xIn + yIn + rotation;
	    wheelSpeeds[MotorType.kFrontRight.value] = -xIn + yIn - rotation;
	    wheelSpeeds[MotorType.kRearLeft.value] = -xIn + yIn + rotation;
	    wheelSpeeds[MotorType.kRearRight.value] = xIn + yIn - rotation;

	    normalize(wheelSpeeds);
	    m_frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput * multiplier);
	    m_frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput * multiplier);
	    m_rearLeftMotor.set(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput * multiplier);
	    m_rearRightMotor.set(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput * multiplier);
	    

	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	  }

	  /**
	   * Drive method for Mecanum wheeled robots.
	   *
	   * <p>A method for driving with Mecanum wheeled robots. There are 4 wheels on the robot, arranged
	   * so that the front and back wheels are toed in 45 degrees. When looking at the wheels from the
	   * top, the roller axles should form an X across the robot.
	   *
	   * @param magnitude The speed that the robot should drive in a given direction.
	   * @param direction The direction the robot should drive in degrees. The direction and maginitute
	   *                  are independent of the rotation rate.
	   * @param rotation  The rate of rotation for the robot that is completely independent of the
	   *                  magnitute or direction. [-1.0..1.0]
	   */
	  public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
	    if (!kMecanumPolar_Reported) {
	      HAL.report(tResourceType.kResourceType_RobotDrive, getNumMotors(),
	          tInstances.kRobotDrive_MecanumPolar);
	      kMecanumPolar_Reported = true;
	    }
	    // Normalized for full power along the Cartesian axes.
	    magnitude = limit(magnitude) * Math.sqrt(2.0);
	    // The rollers are at 45 degree angles.
	    double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
	    double cosD = Math.cos(dirInRad);
	    double sinD = Math.sin(dirInRad);

	    double[] wheelSpeeds = new double[kMaxNumberOfMotors];
	    wheelSpeeds[MotorType.kFrontLeft.value] = (sinD * magnitude + rotation);
	    wheelSpeeds[MotorType.kFrontRight.value] = (cosD * magnitude - rotation);
	    wheelSpeeds[MotorType.kRearLeft.value] = (cosD * magnitude + rotation);
	    wheelSpeeds[MotorType.kRearRight.value] = (sinD * magnitude - rotation);

	    normalize(wheelSpeeds);
	    m_frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput * maxRPM);
	    m_frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput * maxRPM);
	    m_rearLeftMotor.set(wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput * maxRPM);
	    m_rearRightMotor.set(wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput * maxRPM);

	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	  }

	  
	  /**
	   * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
	   */
	  protected static void normalize(double[] wheelSpeeds) {
	    double maxMagnitude = Math.abs(wheelSpeeds[0]);
	    for (int i = 1; i < kMaxNumberOfMotors; i++) {
	      double temp = Math.abs(wheelSpeeds[i]);
	      if (maxMagnitude < temp) {
	        maxMagnitude = temp;
	      }
	    }
	    if (maxMagnitude > 1.0) {
	      for (int i = 0; i < kMaxNumberOfMotors; i++) {
	        wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
	      }
	    }
	  }
	  
	  /**
	   * Rotate a vector in Cartesian space.
	   */
	  @SuppressWarnings("ParameterName")
	  protected static double[] rotateVector(double x, double y, double angle) {
	    double cosA = Math.cos(angle * (3.14159 / 180.0));
	    double sinA = Math.sin(angle * (3.14159 / 180.0));
	    double[] out = new double[2];
	    out[0] = x * cosA - y * sinA;
	    out[1] = x * sinA + y * cosA;
	    return out;
	  }

	  
	  public void setInvertedMotor(edu.wpi.first.wpilibj.RobotDrive.MotorType kfrontleft, boolean isInverted) {
		    switch (kfrontleft) {
		      case kFrontLeft:
		        m_frontLeftMotor.setInverted(isInverted);
		        break;
		      case kFrontRight:
		        m_frontRightMotor.setInverted(isInverted);
		        break;
		      case kRearLeft:
		        m_rearLeftMotor.setInverted(isInverted);
		        break;
		      case kRearRight:
		        m_rearRightMotor.setInverted(isInverted);
		        break;
		      default:
		        throw new IllegalArgumentException("Illegal motor type: " + kfrontleft);
		    }
		  }
	  
	  /**
	   * Limit motor values to the -1.0 to +1.0 range.
	   */
	  protected static double limit(double num) {
	    if (num > 1.0) {
	      return 1.0;
	    }
	    if (num < -1.0) {
	      return -1.0;
	    }
	    return num;
	  }
	  
	  @Override
	  public void setExpiration(double timeout) {
	    m_safetyHelper.setExpiration(timeout);
	  }

	  @Override
	  public double getExpiration() {
	    return m_safetyHelper.getExpiration();
	  }

	  @Override
	  public boolean isAlive() {
	    return m_safetyHelper.isAlive();
	  }

	  @Override
	  public boolean isSafetyEnabled() {
	    return m_safetyHelper.isSafetyEnabled();
	  }

	  @Override
	  public void setSafetyEnabled(boolean enabled) {
	    m_safetyHelper.setSafetyEnabled(enabled);
	  }

	  @Override
	  public String getDescription() {
	    return "Robot Drive";
	  }

	  @Override
	  public void stopMotor() {
	    if (m_frontLeftMotor != null) {
	      m_frontLeftMotor.set(0);
	    }
	    if (m_frontRightMotor != null) {
	      m_frontRightMotor.set(0);
	    }
	    if (m_rearLeftMotor != null) {
	      m_rearLeftMotor.set(0);
	    }
	    if (m_rearRightMotor != null) {
	      m_rearRightMotor.set(0);
	    }
	    if (m_safetyHelper != null) {
	      m_safetyHelper.feed();
	    }
	  }

	  private void setupMotorSafety() {
	    m_safetyHelper = new MotorSafetyHelper(this);
	    m_safetyHelper.setExpiration(kDefaultExpirationTime);
	    m_safetyHelper.setSafetyEnabled(true);
	  }

	  protected int getNumMotors() {
	    int motors = 0;
	    if (m_frontLeftMotor != null) {
	      motors++;
	    }
	    if (m_frontRightMotor != null) {
	      motors++;
	    }
	    if (m_rearLeftMotor != null) {
	      motors++;
	    }
	    if (m_rearRightMotor != null) {
	      motors++;
	    }
	    return motors;
	  }
	
}
