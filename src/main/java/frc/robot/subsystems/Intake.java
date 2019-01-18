/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpIntake;

/**
 * Intake subsystem of robot. Intakes cargo using top roller. Default command is
 * TeleOpIntake
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.commands.TeleOpIntake
 */
public class Intake extends Subsystem {
	// Commands that use this subsystem.
	TalonSRX intake;

	/**
	 * Constructor. Initializes Talon.
	 */
	public Intake() {
		intake = new TalonSRX(RobotMap.INTAKE);
	}

	/**
	 * Intakes cargo by spinning motor at power
	 * 
	 * @param power power of motor from -1.0 to 1.0
	 */
	public void drive(double power) {
		intake.set(ControlMode.PercentOutput, power);
	}

	/**
	 * Stops intake motor.
	 */
	public void stop() {
		drive(0);
	}

	/**
	 * Initializes the Intake's default command to the TeleOpIntake command. The
	 * default for this subsystem is the associated teliop command.
	 * 
	 * @see frc.robot.commands.TeleOpIntake
	 */
	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new TeleOpIntake());
	}
}
