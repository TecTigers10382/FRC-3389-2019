/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Ejector;

/**
 * Tele-Op command that moves the ejector to eject and retract positions from
 * button on right Joystick.
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.subsystems.Ejector
 */
public class TeleOpEjector extends Command {
	Ejector eject;

	/**
	 * Constructor gains control of the Ejector subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.Ejector
	 */
	public TeleOpEjector() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.eject);
		eject = Robot.eject;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

	}

	/**
	 * When command is run, the claw is moved to slightly open position. and ejector
	 * is ejected
	 * 
	 * @see frc.robot.subsystems.Ejector
	 */
	@Override
	protected void execute() {
		eject.eject();
		Robot.claw.toEject();
	}

	/**
	 * Never allows Ejector command to finish on its own terms.
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Retracts servo if command is ended with isFinished
	 */
	@Override
	protected void end() {
		eject.retract();
	}

	/**
	 * Closes the claw if another command is run that needs it.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
