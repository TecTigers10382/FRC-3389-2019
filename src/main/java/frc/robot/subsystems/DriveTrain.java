/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpDrive;

/**
 * Subsystem for the drive train of the robot. Contains the 4 talons necessary
 * for the operation of all the motors in order to move the robot.
 *
 * @author FRC Team 3389 TEC Tigers
 * 
 */
public class DriveTrain extends Subsystem {
	// Commands that use this subsystem.
	protected static final int kMaxNumberOfMotors = 4;
	protected double m_maxOutput = 1;
	public TalonSRX leftFront;
	public TalonSRX leftRear;
	public TalonSRX rightFront;
	public TalonSRX rightRear;

	/**
	 * Creates the Drive Train with 4 TalonSRX motor controllers over CAN.
	 */
	public DriveTrain() {
		leftFront = new TalonSRX(RobotMap.DRIVE_LEFTFRONT);
		leftRear = new TalonSRX(RobotMap.DRIVE_LEFTREAR);
		rightFront = new TalonSRX(RobotMap.DRIVE_RIGHTFRONT);
		rightRear = new TalonSRX(RobotMap.DRIVE_RIGHTREAR);
	}

	/**
	 * Drives the Drive Train with 1 analog stick's x & y values to move forward & backward and
	 * strafe left & right. Another analog stick's x values determine rotation.
	 * 
	 * 
	 * @param x
	 *            value of left stick x from -1.0 to 1.0
	 * @param y
	 *            value of left stick y from -1.0 to 1.0
	 * @param rotation
	 *            value of right stick x from -1.0 to 1.0
	 */
	public void mecanumDrive_Cartesian(double x, double y, double rotation) {

		double xIn = x;
		double yIn = y;
		// Negate y for the joystick.
		yIn = -yIn;

		double[] wheelSpeeds = new double[kMaxNumberOfMotors];
		wheelSpeeds[RobotMap.DRIVE_LEFTFRONT] = xIn + yIn + rotation;
		wheelSpeeds[RobotMap.DRIVE_RIGHTFRONT] = -xIn + yIn - rotation;
		wheelSpeeds[RobotMap.DRIVE_LEFTREAR] = -xIn + yIn + rotation;
		wheelSpeeds[RobotMap.DRIVE_RIGHTREAR] = xIn + yIn - rotation;

		normalize(wheelSpeeds);
		leftFront.set(ControlMode.PercentOutput, wheelSpeeds[MotorType.kFrontLeft.value] * m_maxOutput);
		leftRear.set(ControlMode.PercentOutput, wheelSpeeds[MotorType.kFrontRight.value] * m_maxOutput);
		rightFront.set(ControlMode.PercentOutput, wheelSpeeds[MotorType.kRearLeft.value] * m_maxOutput);
		rightRear.set(ControlMode.PercentOutput, wheelSpeeds[MotorType.kRearRight.value] * m_maxOutput);
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
	 * Initializes the DriveTrain's default command to the Drive command. The
	 * default for this subsystem is the associated teliop command.
	 * 
	 * @see frc.robot.commands.TeleOpDrive
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TeleOpDrive());

	}
}
