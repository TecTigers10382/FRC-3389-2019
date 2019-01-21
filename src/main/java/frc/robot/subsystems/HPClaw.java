/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.TeleOpHP;

/**
 * Put Description here.
 *
 * @author FRC Team 3389 TEC Tigers
 */
public class HPClaw extends Subsystem {
	// Commands that use this subsystem.
	Servo claw;

	public HPClaw() {
		claw = new Servo(1);
	}

	public void close() {
		claw.setAngle(0);
	}

	public void open() {
		claw.setAngle(30);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new TeleOpHP());
	}
}
