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
	NetworkTable input;

	final int IMAGE_WIDTH = 128;
	final int IMAGE_HEIGHT = 64;
	// degrees
	final double FOV_H = 61;
	final double FOCAL_LENGTH = IMAGE_WIDTH / (2 * Math.atan(Math.toRadians(FOV_H) / 2));

	final double CAMERA_X = 0;
	final double CAMERA_Y = 0;
	final double CAMERA_Z = 0;
	final double CAMERA_YAW = 0;
	final double CAMERA_PITCH = 0;
	// Please do not roll the camera I don't account for it.
	// If you want to figure out the math for me please.
	final double CAMERA_ROLL = 0;

	final double TARGET_HEIGHT = 3 * 12 + 3 + 1.0 / 8.0 - 5.5 / 2.0;

	/**
	 * Stores camera location relative to robot.
	 */
	Pose cameraLocation;

	/**
	 * Stores target location relative to camera.
	 */
	Vector3d targetL;
	Vector3d targetR;

	/**
	 * Stores target location relative to robot.
	 */
	Vector3d targetC;

	Vector3d line;

	public VisionCargoBay(NetworkTable table) {
		input = table;
		cameraLocation = new Pose(CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL);
		targetL = new Vector3d();
		targetC = new Vector3d();
		line = new Vector3d();
	}

	/**
	 * Processes data from network tables to update the target location relative to
	 * the robot.
	 */
	void processData() {
		double[] defaultValue = null;
		double[] areas = input.getEntry("area").getDoubleArray(defaultValue);
		double[] centerX = input.getEntry("centerX").getDoubleArray(defaultValue);
		double[] centerY = input.getEntry("centerY").getDoubleArray(defaultValue);

		if (areas != null && areas.length != 1) {
			double cXLeft, cXRight, cY;
			if (areas.length > 2) {
				System.out.println("Woah nelly there's a lot of targets here!");
				// TODO find way to only get center target.
				cXLeft = 0;
				cXRight = 0;
				cY = 0;
			} else {
				if (centerX[1] > centerX[0]) {
					cXLeft = centerX[0];
					cXRight = centerX[1];
				} else {
					cXLeft = centerX[1];
					cXRight = centerX[0];
				}
				cY = (centerY[0] + centerY[1]) / 2;
			}
			// Gives angle relative to center y line, left is negative
			double yawLeft = Math.atan((cXLeft - IMAGE_WIDTH / 2) / FOCAL_LENGTH);
			double yawRight = Math.atan((cXRight - IMAGE_WIDTH / 2) / FOCAL_LENGTH);
			// Gives angle relative to center x line, down is negative
			double pitch = Math.atan((cY - IMAGE_HEIGHT / 2) / FOCAL_LENGTH);

			// Rotate target angles to robot's reference frame
			yawLeft = yawLeft + cameraLocation.yaw;
			yawRight = yawRight + cameraLocation.yaw;
			pitch = pitch + cameraLocation.pitch;

			targetL.set(Math.sin(yawLeft) * Math.cos(pitch), Math.cos(yawLeft) * Math.cos(pitch), Math.sin(pitch));
			targetL.normalize();

			targetR.set(Math.sin(yawRight) * Math.cos(pitch), Math.cos(yawRight) * Math.cos(pitch), Math.sin(pitch));
			targetR.normalize();
			// r(t) = <0,0,0> + t*(targetLocation);
			// plane: z = TARGET_HEIGHT;
			// line : z = t*targetLocation.z;
			// t = TARGET_HEIGHT/targetLocation.z;
			targetL.scale((TARGET_HEIGHT - cameraLocation.z) / targetL.z);
			targetR.scale((TARGET_HEIGHT - cameraLocation.z) / targetR.z);

			// By having the point of the left and right side we have a line segment in
			// space of the target.

			// Sets position of target relative to the center of the robot
			targetC.set((targetL.x + targetR.x) / 2 + cameraLocation.x, (targetL.y + targetR.y) / 2 + cameraLocation.y,
					(targetL.z + targetR.z) / 2 + cameraLocation.z);

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
		return Math.atan(line.y / line.x);
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
