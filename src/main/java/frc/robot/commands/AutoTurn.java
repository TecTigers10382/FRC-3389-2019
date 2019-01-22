/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class AutoTurn extends Command {
	// Set up PID constants
	double kP = 0.0005, kI = 0.00001, kD = 0, kF = 0;

	double integral = 0, lastError, target, error, power;
	long lastTime, time;

	PIDSource input;

	boolean done = false;

	public AutoTurn(PIDSource input, double target) {
		//super("turn", kP, kI, kD, kF);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.input = input;
		this.target = target;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.driveTrain.resetGyro();
		lastError = target - input.pidGet();
		lastTime = System.currentTimeMillis();
		kP = Robot.prefs.getDouble("kP", kP);
		SmartDashboard.putNumber("kP", kP);
		kI = Robot.prefs.getDouble("kI", kI);
		kD = Robot.prefs.getDouble("kD", kD);
		kF = Robot.prefs.getDouble("kF", kF);
	}
	
	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Initializes
		power = 0;
		error = target - input.pidGet();
		SmartDashboard.putNumber("error", error);
		time = System.currentTimeMillis();

		// Calculates running integral
		integral += error * (time - lastTime);

		// Calculates derivative
		double derivative = ((error - lastError) / (time - lastTime));
		SmartDashboard.putNumber("derivative", derivative);

		// Calculates output power to motor
		power = kP * error + kI * integral + (kD * derivative) + kF * target;
		SmartDashboard.putNumber("power", power);

		// Writes power to motor
		Robot.driveTrain.mecanumDrive_Cartesian(0, 0, -power);

		// Sets up variables for next time
		lastTime = time;
		lastError = error;

		// Checks if it has reached target
		if (Math.abs(error) < 1 && Math.abs(power) < .01)
			done = true;
		// try {
		// wait(250);
		// } catch (InterruptedException e) {
		// // TODO Auto-generated catch block
		// e.printStackTrace();
		// }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.driveTrain.mecanumDrive_Cartesian(0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.driveTrain.mecanumDrive_Cartesian(0, 0, 0);
	}

// 	@Override
// 	protected double returnPIDInput() {
// 		return input.pidGet();
// 	}

// 	@Override
// 	protected void usePIDOutput(double output) {
// 		Robot.driveTrain.mecanumDrive_Cartesian(0, 0, -output);
// 	}
 }
