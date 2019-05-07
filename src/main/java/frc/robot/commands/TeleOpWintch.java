/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Wintch;

/**
 * Tele-Op command that moves the claw to open and closed positions based on
 * button on right Joystick.
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.subsystems.HPClaw
 */
public class TeleOpWintch extends Command {
	Wintch wintch;
	double speed;

	/**
	 * Constructor gains control of the HPClaw subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.HPClaw
	 */
	public TeleOpWintch(double power) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.wintch);
		wintch = Robot.wintch;
		speed = power;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	/**
	 * When command is run, the claw is moved to open position.
	 * 
	 * @see frc.robot.subsystems.HPClaw
	 */
	@Override
	protected void execute() {
		wintch.run(speed);
	}

	/**
	 * Never allows claw command to finish on its own terms.
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Closes the claw if command is ended with isFinished
	 */
	@Override
	protected void end() {
		wintch.stop();
	}

	/**
	 * Closes the claw if another command is run that needs it.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
