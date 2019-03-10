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
import frc.robot.subsystems.CargoHold;

/**
 * Tele-Op command that continuously updates the Intakes with the values from
 * the right Joystick
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.subsystems.Intake
 */
public class TeleOpIntake extends Command {
	double rollerSpeed, cargoSpeed;
	CargoHold cargo;
	Intake intake;

	/**
	 * Constructor gains control of the Intake subsystem of the robot.
	 * 
	 * @see frc.robot.subsystems.Intake
	 */
	public TeleOpIntake(double roller, double cargoPower) {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.intake);
		requires(Robot.cargo);

		intake = Robot.intake;
		cargo = Robot.cargo;
		rollerSpeed = roller;
		cargoSpeed = cargoPower;

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
		intake.drive(rollerSpeed);
		cargo.run(-cargoSpeed);
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
		cargo.stop();
	}

	/**
	 * Stops intake's motion if another command is ran that needs it.
	 */
	@Override
	protected void interrupted() {
		end();
	}
}
