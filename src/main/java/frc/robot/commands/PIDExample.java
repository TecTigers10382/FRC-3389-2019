/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;

public class PIDExample extends Command {
	// Set up PID constants
	final double kP = 0, kI = 0, kD = 0;

	double integral = 0, lastError, target, error, power;
	long lastTime, time;

	PIDSource input;
	PIDOutput out;

	boolean done = false;

	public PIDExample(PIDSource input, PIDOutput output, double target) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.input = input;
		this.target = target;
		out = output;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		lastError = target - input.pidGet();
		lastTime = System.currentTimeMillis();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Initializes
		power = 0;
		error = target - input.pidGet();
		time = System.currentTimeMillis();

		// Calculates running integral
		integral += error * (time - lastTime);

		// Calculates output power to motor
		power = kP * error + kI * integral + kD * ((error - lastError) / (time - lastTime));

		// Writes power to motor
		out.pidWrite(power);

		// Sets up variables for next time
		lastTime = time;
		lastError = error;

		// Checks if it has reached target
		if (Math.abs(error) < 1)
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
		out.pidWrite(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		out.pidWrite(0);
	}
}
