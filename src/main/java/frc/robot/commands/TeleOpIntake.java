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
import frc.robot.subsystems.Intake;

/**
 * Tele-Op command that continuously updates the Intakes with the values from
 * the right Joystick
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.subsystems.Intake
 */
public class TeleOpIntake extends Command {

	Joystick intakeStick;
	Intake intake;

	/**
	 * Constructor gains control of the Intake subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.Intake
	 */
	public TeleOpIntake() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.intake);
		intake = Robot.intake;

		intakeStick = Robot.operatorControllers.getOperatorJoystick();
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
	 * As the command is run, updates the joystick values and controls the Intake
	 * with them.
	 * 
	 * @see frc.robot.subsystems.Intake
	 */
	@Override
	protected void execute() {
		double power = intakeStick.getRawAxis(1);
		
		if(Math.abs(power) < RobotMap.DEADZONE)
			intake.drive(0);
		else
			intake.drive(power);
	}

	/**
	 * Never allows IntakeStick command to finish on its own terms.
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

	/**
	 * Stops intake's motion if command is ended with isFinished
	 * 
	 */
	@Override
	protected void end() {
		intake.stop();
	}

	/**
	 * Stops intake's motion if another command is ran that needs it.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
