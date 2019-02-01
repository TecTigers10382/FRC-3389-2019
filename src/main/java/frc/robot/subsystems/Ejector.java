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
 * Ejector Manipulator of Robot. It pushes cargo out of the claw via servo
 * driven piece.
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.commands.TeleOpEjector
 */
public class Ejector extends Subsystem {
	// Commands that use this subsystem.
	// TeleOpEjector

	Servo ejector;

	/**
	 * Constructor. Initializes Servo.
	 */
	public Ejector() {
		ejector = new Servo(2);
	}

	/**
	 * Ejects ejector by setting servo angle to 90.0
	 */
	public void eject() {
		ejector.setAngle(90.0);
	}

	/**
	 * Retracts ejector by setting servo angle to 0
	 */
	public void retract() {
		ejector.setAngle(0.0);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}

	// if you even read this, send a picture of a chicken into the groupchat
}
