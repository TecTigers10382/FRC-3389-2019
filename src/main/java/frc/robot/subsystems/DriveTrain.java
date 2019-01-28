/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;
import frc.robot.iodevices.GyroWrapper;

/**
 * Subsystem for the drive train of the robot. Contains the 4 talons necessary
 * for the operation of all the motors in order to move the robot.
 *
 * @author FRC Team 3389 TEC Tigers
 * 
 */
public class DriveTrain extends Subsystem {
	// Commands that use this subsystem.
	// TeleOpDrive

	protected static final int kMaxNumberOfMotors = 4;
	protected double m_maxOutput = 1;
	public TalonSRX leftFront;
	public TalonSRX leftRear;
	public TalonSRX rightFront;
	public TalonSRX rightRear;

	public DigitalInput line;

	public GyroWrapper gyro;

	/**
	 * Creates the Drive Train with 4 TalonSRX motor controllers over CAN.
	 */
	public DriveTrain() {
		leftFront = new TalonSRX(RobotMap.DRIVE_LEFTFRONT);
		leftRear = new TalonSRX(RobotMap.DRIVE_LEFTREAR);
		rightFront = new TalonSRX(RobotMap.DRIVE_RIGHTFRONT);
		rightRear = new TalonSRX(RobotMap.DRIVE_RIGHTREAR);

		leftFront.setInverted(false);
		leftRear.setInverted(false);
		rightFront.setInverted(true);
		rightRear.setInverted(true);

		line = new DigitalInput(0);

		gyro = new GyroWrapper();
		gyro.startUpdatingThread();
		gyro.resetValues();
	}

	/**
	 * Drives the Drive Train with 1 analog stick's x & y values to move forward &
	 * backward and strafe left & right. Another analog stick's x values determine
	 * rotation.
	 * 
	 * 
	 * @param x        value of left stick x from -1.0 to 1.0
	 * @param y        value of left stick y from -1.0 to 1.0
	 * @param rotation value of right stick x from -1.0 to 1.0
	 */
	public void mecanumDrive_Cartesian(double x, double y, double rotation) {

		double xIn = x;
		double yIn = y;
		// Negate y for the joystick.
		yIn = -yIn;

		double[] wheelSpeeds = new double[kMaxNumberOfMotors];
		wheelSpeeds[0] = xIn + yIn + rotation;
		wheelSpeeds[1] = -xIn + yIn - rotation;
		wheelSpeeds[2] = -xIn + yIn + rotation;
		wheelSpeeds[3] = xIn + yIn - rotation;

		normalize(wheelSpeeds);
		leftFront.set(ControlMode.PercentOutput, wheelSpeeds[0] * m_maxOutput);
		rightFront.set(ControlMode.PercentOutput, wheelSpeeds[1] * m_maxOutput);
		leftRear.set(ControlMode.PercentOutput, wheelSpeeds[2] * m_maxOutput);
		rightRear.set(ControlMode.PercentOutput, wheelSpeeds[3] * m_maxOutput);

		SmartDashboard.putNumber("Speeds FL", wheelSpeeds[0]);
		SmartDashboard.putNumber("Speeds FR", wheelSpeeds[1]);
		SmartDashboard.putNumber("Speeds RL", wheelSpeeds[2]);
		SmartDashboard.putNumber("Speeds RR", wheelSpeeds[3]);
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
	protected static double[] rotateVector(double x, double y, double angle) {
		double cosA = Math.cos(angle * (Math.PI / 180.0));
		double sinA = Math.sin(angle * (Math.PI / 180.0));
		double[] out = new double[2];
		out[0] = x * cosA - y * sinA;
		out[1] = x * sinA + y * cosA;
		return out;
	}

	/**
	 * Stops all Talons.
	 */
	public void stop() {
		mecanumDrive_Cartesian(0, 0, 0);
	}

	/**
	 * Configs all talons to factory defaults and then to the selected
	 * configuration.
	 * 
	 * @param t A configuration for all Talon SRXs in the subsystem.
	 */
	public void configTalons(TalonSRXConfiguration t) {
		leftFront.configFactoryDefault();
		rightFront.configFactoryDefault();
		leftRear.configFactoryDefault();
		rightRear.configFactoryDefault();

		leftFront.configAllSettings(t);
		rightFront.configAllSettings(t);
		leftRear.configAllSettings(t);
		rightRear.configAllSettings(t);
	}

	public void driveVelocity(double leftVelocity, double rightVelocity) {

		double rightVelo = rightVelocity * 4096 * 500 / 600;
		double leftVelo = leftVelocity * 4096 * 500 / 600;
		rightFront.set(ControlMode.Velocity, rightVelo / 1.5);
		leftFront.set(ControlMode.Velocity, leftVelo / 1.5);
		rightRear.set(ControlMode.Velocity, rightVelo / 1.5);
		leftRear.set(ControlMode.Velocity, leftVelo / 1.5);
	}

	/**
	 * Initializes the DriveTrain's default command to the Drive command. The
	 * default for this subsystem is the associated teliop command.
	 * 
	 * @see frc.robot.commands.TeleOpDrive
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TeleOpDrive());

	}

	public boolean getLine() {
		return line.get();
	}

	public GyroWrapper getGyro() {
		return gyro;
	}

	public void resetGyro() {
		gyro.resetValues();
	}
}
