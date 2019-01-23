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
	Vector3d targetLocation;

	/**
	 * Stores target location relative to robot.
	 */
	Vector3d target;

	public VisionCargoBay(NetworkTable table) {
		input = table;
		cameraLocation = new Pose(CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL);
		targetLocation = new Vector3d();
		target = new Vector3d();
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
			double cX, cY;
			if (areas.length > 2) {
				System.out.println("Woah nelly there's a lot of targets here!");
				// TODO find way to only get center target.
				cX = 0;
				cY = 0;
			} else {
				if (centerX[1] > centerX[0]) {
					cX = centerX[1] - centerX[0];
				} else {
					cX = centerX[0] - centerX[1];
				}
				cY = (centerY[0] + centerY[1]) / 2;
			}
			// Gives angle relative to center y line, left is negative
			double yaw = Math.atan((cX - IMAGE_WIDTH / 2) / FOCAL_LENGTH);
			// Gives angle relative to center x line, down is negative
			double pitch = Math.atan((cY - IMAGE_HEIGHT / 2) / FOCAL_LENGTH);

			// Rotate target angles to robot's reference frame
			yaw = yaw + cameraLocation.yaw;
			pitch = pitch + cameraLocation.pitch;

			targetLocation.set(Math.sin(yaw) * Math.cos(pitch), Math.cos(yaw) * Math.cos(pitch), Math.sin(pitch));
			targetLocation.normalize();
			// r(t) = <0,0,0> + t*(targetLocation);
			// plane: z = TARGET_HEIGHT;
			// line : z = t*targetLocation.z;
			// t = TARGET_HEIGHT/targetLocation.z;
			targetLocation.scale((TARGET_HEIGHT - cameraLocation.z) / targetLocation.z);

			// Sets position of target relative to the center of the robot
			target.set(targetLocation.x + cameraLocation.x, targetLocation.y + cameraLocation.y,
					targetLocation.z + cameraLocation.z);
		} else {
			System.out.println("ERROR Camera failed to find any targets");
		}
	}

	/**
	 * @return Distance from center of robot to target projected onto XY plane.
	 */
	public double getDistanceXY() {
		return Math.sqrt(Math.pow(target.x, 2) + Math.pow(target.y, 2));
	}

	/**
	 * @return Degress relative to robot y axis to target. CW is positive.
	 */
	public double yawDegrees() {
		return Math.toDegrees(Math.atan(target.x / target.y));
	}
}
