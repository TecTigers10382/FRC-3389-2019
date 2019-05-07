/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpWintch;

/**
 * Hatch Panel Manipulator of robot. Grabs panels by opening claw inside panel
 * opening.
 *
 * @author FRC Team 3389 TEC Tigers
 * @see frc.robot.commands.TeleOpHP
 */
public class Wintch extends Subsystem {
	// Commands that use this subsystem.
	// TeleOpHP

	TalonSRX wintch;

	/**
	 * Constructor. Initializes Servo.
	 */
	public Wintch() {
		wintch = new TalonSRX(RobotMap.WINTCH);
	}

	public void run(double power) {
		wintch.set(ControlMode.PercentOutput, power);
	}

	public void stop() {
		wintch.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}
}
