/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.vision;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

/**
 * Holds a location in space.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class Pose {
	public final double x, y, z, yaw, pitch, roll;

	public Pose(double x, double y, double z, double alpha, double beta, double gamma) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.yaw = alpha;
		this.pitch = beta;
		this.roll = gamma;
	}

	public Pose() {
		this(0, 0, 0, 0, 0, 0);
	}

	public Pose setXYZ(Vector3d v) {
		return new Pose(v.x, v.y, v.z, yaw, pitch, roll);
	}

	public Pose changeOrigin(Point3d p) {
		return new Pose(x - p.x, y - p.y, z - p.z, yaw, pitch, roll);
	}

	public Point3d getPoint() {
		return new Point3d(x, y, z);
	}
}
