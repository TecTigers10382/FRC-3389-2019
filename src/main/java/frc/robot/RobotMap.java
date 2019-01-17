/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class RobotMap {

	/**
	 * CAN IDs of all Talon SRXs used for drive train
	 */
	public final static int DRIVE_LEFTFRONT = 4, DRIVE_LEFTREAR = 0, DRIVE_RIGHTFRONT = 3, DRIVE_RIGHTREAR = 1;
	
	public final static int INTAKE = 2;


	public static final int LEFT_X_STICK = 0, LEFT_Y_STICK = 1, RIGHT_DRIVE_STICK = 4;
	


	public static final double DEADZONE = 0.1;
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
