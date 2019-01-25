/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.iodevices;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class AnalogUltraSonic implements PIDSource {
	AnalogInput ultra;
	PIDSourceType sourceType = PIDSourceType.kDisplacement;
	double conversionFactor = 20.0 / .4907;

	public AnalogUltraSonic(int port) {
		ultra = new AnalogInput(port);
	}

	public double getDistanceInches() {
		return ultra.getAverageVoltage() * conversionFactor;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		sourceType = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		return getDistanceInches();
	}
}
