/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import javax.vecmath.Vector3d;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.utils.vision.Pose;

/**
 * Uses network table output from GRIP and converts it into robot coordinates
 * and other utils.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class VisionCargoBay {
	// All units are in inches and radians unless otherwise noted.

	NetworkTable input;

	final int IMAGE_WIDTH = 128;
	final int IMAGE_HEIGHT = 64;
	// camera FOV in degrees.
	final double FOV_H = 61;
	// Uses FOV to find the focal length of the camera.
	final double FOCAL_LENGTH = IMAGE_WIDTH / (2 * Math.atan(Math.toRadians(FOV_H) / 2));

	final double CAMERA_X = 0;
	final double CAMERA_Y = 0;
	final double CAMERA_Z = 0;
	final double CAMERA_YAW = 0;
	final double CAMERA_PITCH = 0;
	// Please do not roll the camera I don't account for it.
	// If you want to, figure out the math for me please.
	final double CAMERA_ROLL = 0;

	/**
	 * Height of target center in inches measured from the ground.
	 */
	final double TARGET_HEIGHT = 3 * 12 + 3 + 1.0 / 8.0 - 5.5 / 2.0;

	/**
	 * Stores camera location relative to robot.
	 */
	Pose cameraLocation;

	/**
	 * Stores target location relative to robot.
	 */
	Vector3d targetL, targetR, targetC;

	/**
	 * Stores a vector from the left side of the target to the right.
	 */
	Vector3d line;

	/**
	 * @param table A network table that contains a contour report.
	 */
	public VisionCargoBay(NetworkTable table) {
		input = table;
		cameraLocation = new Pose(CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL);
		targetL = new Vector3d();
		targetR = new Vector3d();
		targetC = new Vector3d();
		line = new Vector3d();
	}

	/**
	 * Processes data from network tables to update the target location relative to
	 * the robot.
	 */
	void processData() {
		double[] defaultValue = new double[0];
		double[] centerX = input.getEntry("centerX").getDoubleArray(defaultValue);
		double[] centerY = input.getEntry("centerY").getDoubleArray(defaultValue);

		// Checks that the camera has actually found the target.
		if (centerX.length >= 2) {
			double cXLeft, cXRight, cY;
			// Checks that there isn't too many targets, and hopefully in the future will
			// find the middle target and approach that one.
			if (centerX.length > 2) {
				System.out.println("Woah nelly there's a lot of targets here!");
				// TODO find way to only get center target so it's not just erroring.
				cXLeft = 0;
				cXRight = 0;
				cY = 0;
			} else {
				// Makes sure the left side is on the left.
				if (centerX[1] > centerX[0]) {
					cXLeft = centerX[0];
					cXRight = centerX[1];
				} else {
					cXLeft = centerX[1];
					cXRight = centerX[0];
				}
				// (As long as there is no roll) the targets should have the same Y coordinate.
				// Since we have two might as well take the average to make sure the Y is as
				// close as possible.
				cY = (centerY[0] + centerY[1]) / 2;
			}
			// Gives angle in radians relative to center y line, left is negative
			double yawLeft = Math.atan((cXLeft - IMAGE_WIDTH / 2) / FOCAL_LENGTH);
			double yawRight = Math.atan((cXRight - IMAGE_WIDTH / 2) / FOCAL_LENGTH);
			// Gives angle in radians relative to center x line, down is negative
			double pitch = Math.atan((cY - IMAGE_HEIGHT / 2) / FOCAL_LENGTH);

			// Rotate target angles to robot's reference frame
			yawLeft = yawLeft + cameraLocation.yaw;
			yawRight = yawRight + cameraLocation.yaw;
			pitch = pitch + cameraLocation.pitch;

			// Using some trig (see MAGIC), converts the pitch and yaw to unit vectors in
			// the direction of the target for each side.
			targetL.set(Math.sin(yawLeft) * Math.cos(pitch), Math.cos(yawLeft) * Math.cos(pitch), Math.sin(pitch));
			targetR.set(Math.sin(yawRight) * Math.cos(pitch), Math.cos(yawRight) * Math.cos(pitch), Math.sin(pitch));

			// ***JUST*** in case they aren't actually unit vectors (which is impossible),
			// normalizes vectors.
			targetL.normalize();
			targetR.normalize();

			// Finds what factor causes the unit vectors to intersect with with the plane..
			// z = TARGET_HEIGHT
			//
			// equation of line: r(t) = t*(targetLocation);
			// plane (relative to camera): z = TARGET_HEIGHT - cameraLocation.z;
			// z part of line: z = t*targetLocation.z;
			// t = (TARGET_HEIGHT-cameraLocation.z)/targetLocation.z;
			// Scaling the vector by this t will make the end of it where the target is in 3
			// space.
			targetL.scale((TARGET_HEIGHT - cameraLocation.z) / targetL.z);
			targetR.scale((TARGET_HEIGHT - cameraLocation.z) / targetR.z);

			// Sets position of target relative to the center of the robot.
			targetC.set((targetL.x + targetR.x) / 2 + cameraLocation.x, (targetL.y + targetR.y) / 2 + cameraLocation.y,
					(targetL.z + targetR.z) / 2 + cameraLocation.z);

			// Sets positions of left and right vectors so they are relative to robot
			// center.
			targetL.add(cameraLocation.getPoint());
			targetR.add(cameraLocation.getPoint());

			// By having the point of the left and right side we have a line segment in
			// space of the target.
			// Store projection of the vector from left to right on XY plane.
			line.set(targetR.x - targetL.x, targetR.y - targetL.y, 0);
		} else {
			System.out.println("ERROR Camera failed to find any targets");
		}
	}

	/**
	 * @return Distance from center of robot to target projected onto XY plane.
	 */
	public double getDistanceXY() {
		return Math.sqrt(Math.pow(targetC.x, 2) + Math.pow(targetC.y, 2));
	}

	/**
	 * @return Degrees needed to be rotated until robot x axis is parrallel to
	 *         target face. CCW is positive.
	 */
	public double yawDegrees() {
		return Math.toDegrees(Math.atan(line.y / line.x));
	}

	/**
	 * @return Degrees needed to rotate so the robot is facing the target. CW is
	 *         positive.
	 */
	public double rawDegrees() {
		return Math.toDegrees(Math.atan(targetC.y / targetC.x));
	}

	/**
	 * @return Distance needed to be traveled in the x direction after the robot has
	 *         rotated.
	 */
	public double distanceX() {
		// Angle needed to be turned
		double yaw = Math.atan(line.y / line.x);
		// Rotation of axes
		double xPrimeC = targetC.x * Math.cos(yaw) + targetC.y * Math.sin(yaw);
		return xPrimeC;
	}

	/**
	 * @return Distance needed to be traveled in the y direction after the robot has
	 *         rotated.
	 */
	public double distanceY() {
		// Angle needed to be turned
		double yaw = Math.atan(line.y / line.x);
		// Rotation of axes()
		double yPrimeC = -targetC.x * Math.sin(yaw) + targetC.y * Math.cos(yaw);
		return yPrimeC;
	}
}
