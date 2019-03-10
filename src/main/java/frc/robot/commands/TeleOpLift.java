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
 * @see @see frc.robot.subsystems.Lift
 */
public class TeleOpLift extends Command {
	Joystick joystick;
	Lift lift;

	/**
	 * Constructor gains control of the Lift subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.Lift
	 */
	public TeleOpLift() {
		requires(Robot.lift);
		lift = Robot.lift;
		joystick = Robot.operatorControllers.getOperatorJoystick();
	}

	/**
	 * Grabs the joystick from the OI object of the robot before running the
	 * command.
	 * 
	 */
	@Override
	protected void initialize() {
	}

	/**
	 * As the command is run, updates the joystick values and controls the Lift with
	 * them.
	 * 
	 * @see frc.robot.subsystems.Lift
	 * 
	 */
	@Override
	protected void execute() {
		double power = joystick.getRawAxis(RobotMap.LIFT_STICK);

		if (power > 0)
			lift.rawLift(power * .3);
		else
			lift.rawLift(power);
	}

	/**
	 * Never allows Lift command to finish on its own terms.
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Stops lift's motion if command is ended with isFinished
	 * 
	 */
	@Override
	protected void end() {
		lift.stop();
	}

	/**
	 * Stops lift's motion if another command is run that needs it.
	 * 
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
