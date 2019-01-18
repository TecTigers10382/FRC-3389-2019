/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Lift;

/**
 * Runs the lift using the operator's controller's right stick.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class TeleOpLift extends Command {
	Joystick joystick;
	Lift lift;

	public TeleOpLift() {
		requires(Robot.lift);
		lift = Robot.lift;
		joystick = Robot.operatorControllers.getOperatorJoystick();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		double power = joystick.getRawAxis(RobotMap.LIFT_STICK);

		if (Math.abs(power) < RobotMap.DEADZONE)
			lift.rawLift(0);
		else
			lift.rawLift(power);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		lift.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
