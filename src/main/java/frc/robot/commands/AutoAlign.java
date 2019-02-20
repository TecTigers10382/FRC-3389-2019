/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

/**
 * Put description here.
 *
 * @author FRC Team 3389 TEC Tigers
 */
public class AutoAlign extends Command {
	double[] pos = new double[4];
	DriveTrain drive;
	boolean done = false;

	public AutoAlign() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.drive_Train);
		drive = new DriveTrain();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		if (Robot.bay.getTargetID()) {
			pos = Robot.bay.mecanumPath(Robot.bay.deltaX(), Robot.bay.deltaY(), Robot.bay.yawDegrees());
			drive.drivePosition(pos[0], pos[1], pos[2], pos[3]);
		} else
			done = true;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		drive.mecanumDrive_Cartesian(0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
