/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

/**
 * Tele-Op command that continuously updates the DriveTrain with the values from
 * the left Joystick
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.subsystems.DriveTrain
 * 
 */
public class TeleOpDrive extends Command {
	Joystick driveStick;
	DriveTrain drive;

	/**
	 * Constructor gains control of the DriveTrain subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.DriveTrain
	 */
	public TeleOpDrive() {
		// Use requires() here to declare subsystem dependencies
		// requires(Robot.driveTrain);
		drive = Robot.drive_Train;

		driveStick = Robot.operatorControllers.getDriverJoystick();
	}

	/**
	 * Initialize is called each time the command is going to be used
	 * 
	 */
	@Override
	protected void initialize() {
		drive.configRamp();
	}

	/**
	 * As the command is run, updates the joystick values and controls the
	 * DriveTrain with them.
	 * 
	 * @see frc.robot.subsystems.DriveTrain
	 */
	@Override
	protected void execute() {
		double x = driveStick.getRawAxis(RobotMap.LEFT_X_STICK);
		double y = driveStick.getRawAxis(RobotMap.LEFT_Y_STICK);
		double rotation = driveStick.getRawAxis(RobotMap.RIGHT_DRIVE_STICK);

		if (Math.abs(x) < RobotMap.DEADZONE)
			x = 0;
		if (Math.abs(y) < RobotMap.DEADZONE)
			y = 0;
		if (Math.abs(rotation) < RobotMap.DEADZONE)
			rotation = 0;

		drive.mecanumDrive_Cartesian(x, y, rotation);
	}

	/**
	 * Never allows Drive command to finish on its own terms.
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Stops drivetrain's motion if command is ended with isFinished
	 * 
	 */
	@Override
	protected void end() {
		drive.stop();
	}

	/**
	 * Stops drivetrain's motion if another command is ran that needs it.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
