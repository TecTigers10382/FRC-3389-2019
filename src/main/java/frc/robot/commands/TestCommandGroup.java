/*
* Copyright (c) 2017-2018 FRC TEAM 3389. All Rights Reserved.
* Open Source Software - may be modified and shared by FRC teams. The code
* must be accompanied by the FIRST BSD license file in the root directory of
* the project.
*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.utils.VisionCargoBay;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class TestCommandGroup extends CommandGroup {
	// private properties here
	
	public TestCommandGroup() {
		if(Robot.bay.getTargetID()){
			//FIRST: get target to the center
				addSequential(new DriveTurn(.5 ,Robot.bay.rawDegrees()));//assuming gyro returns degrees
			//SECOND: get robot parallel
				//addSequential(new DriveTurn(speed, turn));
			//THIRD: strafe delta X
			//FOURTH: drive forward delta Y
			
		}
		// addSequential(new Command());

		// addParallel(new Command());
		// addSequential(new Command());
	}
}
