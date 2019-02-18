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

	Servo clawL, clawR;

	/**
	 * Constructor. Initializes Servo.
	 */
	public HPClaw() {
		clawL = new Servo(1);
		clawR = new Servo(2);
	}

	/**
	 * Closes claw by setting servo angle to 0
	 */
	public void close() {
		clawL.setAngle(0);
		clawR.setAngle(0);
	}

	/**
	 * Opens claw by setting servo angle to 30
	 */
	public void open() {
		clawL.setAngle(180);
		clawR.setAngle(180);
	}

	public void toEject() {
		clawL.setAngle(15);
		clawR.setAngle(15);
	}

	public double getLAngle() {
		return clawL.getAngle();
	}

	public double getRAngle() {
		return clawR.getAngle();
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}
}
