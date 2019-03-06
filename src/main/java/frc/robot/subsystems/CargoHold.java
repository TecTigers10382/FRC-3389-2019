/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Put Description here.
 *
 * @author FRC Team 3389 TEC Tigers
 */
public class CargoHold extends Subsystem {
	// Commands that use this subsystem.
	VictorSP cargo;

	public CargoHold() {
		cargo = new VictorSP(4);
	}

	public void run(double power) {
		cargo.set(power);
	}

	public void stop() {
		run(0);
	}

	@Override
	public void initDefaultCommand() {
		// setDefaultCommand(new MySpecialCommand());
	}
}
