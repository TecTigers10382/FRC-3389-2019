/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class PIDControllerExample extends Command {
	// Set up PID constants
	double kP = 0.0005, kI = 0.00001, kD = 0, kF = 0;
	final double TOLERANCE = 5;
	final int BUFLENGTH = 10;
	PIDController pidController;

	boolean done = false;

	final PIDOutput output = this::usePIDOutput;

	final PIDSource source = new PIDSource() {
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return returnPIDInput();
		}
	};

	public PIDControllerExample() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		pidController = new PIDController(kP, kI, kD, kF, source, output);
		pidController.setPercentTolerance(TOLERANCE);
		pidController.setOutputRange(-1, 1);
		pidController.setToleranceBuffer(BUFLENGTH);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.drive_Train.resetGyro();

		// Gets Parameters from SmartDashboard
		kP = Robot.prefs.getDouble("kP", kP);
		kI = Robot.prefs.getDouble("kI", kI);
		kD = Robot.prefs.getDouble("kD", kD);
		kF = Robot.prefs.getDouble("kF", kF);

		// Initializes Control loop
		pidController.setPID(kP, kI, kD, kF);
		pidController.setSetpoint(getTarget());
		pidController.reset();
		pidController.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		pidController.setSetpoint(getTarget());
		done = pidController.onTarget();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		pidController.disable();
		Robot.drive_Train.mecanumDrive_Cartesian(0, 0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	protected double returnPIDInput() {
		return Robot.drive_Train.getGyro().pidGet();
	}

	protected void usePIDOutput(double output) {
		Robot.drive_Train.mecanumDrive_Cartesian(0, 0, -output);
	}

	protected double getTarget() {
		return 0;
	}
}
