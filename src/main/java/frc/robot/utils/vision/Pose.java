/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.vision;

/**
 * Holds a location in space.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class Pose {
	public double x, y, z, alpha, beta, gamma;

	public Pose(double x, double y, double z, double alpha, double beta, double gamma) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.alpha = alpha;
		this.beta = beta;
		this.gamma = gamma;
	}
}
