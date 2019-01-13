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
 * Put description here.
 *
 * @author FRC Team 3389 TEC Tigers
 */
public class TeleOpDrive extends Command {
	Joystick driveStick;
	DriveTrain drive;

	public TeleOpDrive() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.driveTrain);
		drive = Robot.driveTrain;

		driveStick = Robot.operatorControllers.getDriverJoystick();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double x = driveStick.getRawAxis(RobotMap.LEFT_X_STICK);
		double y = driveStick.getRawAxis(RobotMap.LEFT_Y_STICK);
		double rotation = driveStick.getRawAxis(RobotMap.RIGHT_DRIVE_STICK);

		if (Math.abs(x) < 0.1)
			x = 0;
		if (Math.abs(y) < 0.1)
			y = 0;
		if (Math.abs(rotation) < 0.1)
			rotation = 0;

		drive.mecanumDrive_Cartesian(x, y, rotation);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		drive.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
