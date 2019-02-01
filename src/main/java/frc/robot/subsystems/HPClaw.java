/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Hatch Panel Manipulator of robot. Grabs panels by opening claw inside panel
 * opening.
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.commands.TeleOpHP
 */
public class HPClaw extends Subsystem {
	// Commands that use this subsystem.
	// TeleOpHP

	Servo claw;

	/**
	 * Constructor. Initializes Servo.
	 */
	public HPClaw() {
		claw = new Servo(1);
	}

	/**
	 * Closes claw by setting servo angle to 0
	 */
	public void close() {
		claw.setAngle(0);
	}

	/**
	 * Opens claw by setting servo angle to 30
	 */
	public void open() {
		claw.setAngle(30);
	}

	public void toEject() {
		claw.setAngle(15);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}
}
