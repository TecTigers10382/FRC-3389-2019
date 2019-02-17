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
	 * CAN IDs of all Talon SRXs used for drive train.
	 */
	public final static int DRIVE_LEFTFRONT = 4, DRIVE_LEFTREAR = 0, DRIVE_RIGHTFRONT = 3, DRIVE_RIGHTREAR = 1;

	/**
	 * CAN IDs of all Talon SRXs used for lift.
	 */
	public final static int LIFT_LEFT = 5, LIFT_RIGHT = 6;

	/**
	 * CAN IDs of all Talon SRXs used for intake.
	 */
	public final static int INTAKE = 2;

	/**
	 * IDs of axes on the joysticks.
	 */
	public static final int LEFT_X_STICK = 0, LEFT_Y_STICK = 1, RIGHT_DRIVE_STICK = 4;
	public static final int INTAKE_STICK = 1, LIFT_STICK = 5;

	public static final int POT_INPUT = 3;

	public static final int ULTRA_INPUT = 0;

	/**
	 * Amount of deadzone on the sticks of the joysticks.
	 */
	public static final double DEADZONE = 0.1;

	/**
	 * Max RPM of wheel
	 */
	public static final int kMaxRPM = 250;

	/**
	 * Units per rotation for CIMcoder
	 */
	public static final int kUnitsPerRotation = 80;

	/**
	 * If true, will configure all talons to their factory defaults, then to the
	 * configuration in utils/TalonConfig
	 * 
	 * @see frc.robot.utils.TalonConfig
	 */
	public final static boolean CONFIG_TALONS = false;
}
