/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.RobotMap;
import frc.robot.commands.TeleOpLift;

/**
 * Double reverse four bar lift that has 2 motors.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class Lift extends Subsystem {
	// Commands that use this subsystem
	// TeleOpLift

	TalonSRX liftL, liftR;
	Potentiometer pot;

	double heightRangeLow, heightRangeHigh;

	public Lift() {
		liftL = new TalonSRX(RobotMap.LIFT_LEFT);// Talon 5
		liftR = new TalonSRX(RobotMap.LIFT_RIGHT);// Talon 6

		pot = new AnalogPotentiometer(RobotMap.POT_INPUT, 360, 0);
	}

	/**
	 * Drives the motors of the lift directly by percentage
	 * 
	 * @param power percentage of power to lift motors (-1 to 1)
	 */
	public void rawLift(double power) {
		// Power divided by 2 because it was too fast and caused the lift to be bouncy
		// when moving
		liftR.set(ControlMode.PercentOutput, power / 2);
		liftL.set(ControlMode.PercentOutput, -power / 2);
	}

	public void liftPos(double height) {
		heightRangeLow = height - RobotMap.LIFT_DEADZONE;
		heightRangeHigh = height + RobotMap.LIFT_DEADZONE;
		while (liftR.getSelectedSensorPosition(0) < heightRangeLow
				|| liftR.getSelectedSensorPosition(0) > heightRangeHigh) {
			liftR.set(ControlMode.Position, height);
		}
	}

	/**
	 * Configs all talons to factory defaults and then to the selected
	 * configuration.
	 * 
	 * @param t A configuration for all Talon SRXs in the subsystem.
	 */
	public void configTalons(TalonSRXConfiguration t) {
		liftL.configFactoryDefault();
		liftR.configFactoryDefault();

		liftL.configAllSettings(t);
		liftR.configAllSettings(t);
	}

	/**
	 * Stops lift motors.
	 */
	public void stop() {
		rawLift(0);
	}

	public double getAngle() {
		return pot.get();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new TeleOpLift());
	}
}
