/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CameraTurn extends Command {
	// Set up PID constants
	double kP = 0.0005, kI = 0.00001, kD = 0, kF = 0;

	double integral = 0, lastError, target, error, power;
	long lastTime, time;

	PIDSource input;

	boolean done = false;

	public CameraTurn(PIDSource input, double target) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.drive_Train);
		this.input = input;
		this.target = target;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drive_Train.resetGyro();
		// lastError = target - input.pidGet();
		lastError = getTarget() - input.pidGet();
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
		// error = target - input.pidGet();
		error = getTarget() - input.pidGet();
		System.out.println(error);
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
		Robot.drive_Train.mecanumDrive_Cartesian(0, 0, -power);

		// Sets up variables for next time
		lastTime = time;
		lastError = error;
	}

	private double getTarget() {
		double[] defaultValue = new double[0];
		defaultValue = Robot.lines.getEntry("angle").getDoubleArray(defaultValue);
		if (defaultValue.length > 0) {
			// if (defaultValue[0] < -180) {
			// return defaultValue[0] + 180;
			// }
			return defaultValue[0];
		}

		// done = true;
		System.out.println("ERROR camera sees nothing");
		return 0;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.drive_Train.mecanumDrive_Cartesian(0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		Robot.drive_Train.mecanumDrive_Cartesian(0, 0, 0);
	}
}
