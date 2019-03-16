/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;
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

	VictorSP hook;

	/**
	 * Constructor. Initializes Servo.
	 */
	public HPClaw() {
		hook = new VictorSP(5);
	}

	public void run(double power) {
		hook.set(power);
	}

	public void stop() {
		hook.set(0);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}
}
