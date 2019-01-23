/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.vision;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

/**
 * Holds a location in space.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class Pose {
	public final double x, y, z, yaw, pitch, roll;

	/**
	 * Create a new pose with the specified parameters
	 * 
	 * @param x     x coordinate
	 * @param y     y coordinate
	 * @param z     z coordinate
	 * @param yaw   angle rotated about Z axis
	 * @param pitch angle rotated about X axis
	 * @param roll  angle rotated about Y axis
	 */
	public Pose(double x, double y, double z, double yaw, double pitch, double roll) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.yaw = yaw;
		this.pitch = pitch;
		this.roll = roll;
	}

	/**
	 * Create a new pose at the origin.
	 */
	public Pose() {
		this(0, 0, 0, 0, 0, 0);
	}

	/**
	 * Creates a new pose that is this one but moved to a new position.
	 * 
	 * @param v A point in 3 space.
	 * @return New pose moved to location v.
	 */
	public Pose setXYZ(Tuple3d v) {
		return new Pose(v.x, v.y, v.z, yaw, pitch, roll);
	}

	/**
	 * Creates a new pose that has a shifted origin.
	 * 
	 * @param p Location of new origin.
	 * @return New pose shifted so origin is at point p.
	 */
	public Pose changeOrigin(Tuple3d p) {
		return new Pose(x - p.x, y - p.y, z - p.z, yaw, pitch, roll);
	}

	/**
	 * @return Pose location as a point.
	 */
	public Point3d getPoint() {
		return new Point3d(x, y, z);
	}
}
