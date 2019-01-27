/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;

import javax.vecmath.Vector3d;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.utils.vision.Pose;
import frc.robot.utils.vision.VisionTarget;

/**
 * Uses network table output from GRIP and converts it into robot coordinates
 * and other utils.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class VisionCargoBay {
	// All units are in inches and radians unless otherwise noted.

	NetworkTable contourInput, leftLineInput, rightLineInput;

	final int IMAGE_WIDTH = 640;
	final int IMAGE_HEIGHT = 400;
	// camera FOV in degrees.
	final double FOV_H = 67;
	final double FOV_V = 45;
	// Uses FOV to find the focal length of the camera.
	// final double FOCAL_LENGTH_X = IMAGE_WIDTH / (2 *
	// Math.atan(Math.toRadians(FOV_H) / 2));
	// final double FOCAL_LENGTH_Y = IMAGE_HEIGHT / (2 *
	// Math.atan(Math.toRadians(FOV_V) / 2));

	// Focal Lengths calculated from MATLAB
	final double FOCAL_LENGTH_X = 569.2393;
	final double FOCAL_LENGTH_Y = 576.7241;

	final double CAMERA_X = 0;
	final double CAMERA_Y = 0;
	final double CAMERA_Z = 10 + 6.0 / 8.0;
	final double CAMERA_YAW = 0;
	final double CAMERA_PITCH = Math.toRadians(45);
	// Please do not roll the camera I don't account for it.
	// If you want to, figure out the math for me please.
	final double CAMERA_ROLL = 0;

	// Camera Parameters for Distortion from MATLAB
	final double PRINCIPAL_POINT_X = 323.1647;
	final double PRINCIPAL_POINT_Y = 258.3681;
	// Note: these are negative what comes out of MATLAB
	final double k1 = 0.3173;
	final double k2 = -0.1322;

	/**
	 * Height of target center in inches measured from the ground.
	 */
	final double TARGET_HEIGHT = 3 * 12;

	/**
	 * Stores camera location relative to robot.
	 */
	Pose cameraLocation;

	/**
	 * Stores target location relative to robot.
	 */
	Vector3d targetL, targetR, targetC;
	Vector2d targetCUltraSonic;

	/**
	 * Stores a vector from the left side of the target to the right.
	 */
	Vector3d targetLine;

	/**
	 * @param table A network table that contains a contour report.
	 */
	public VisionCargoBay(NetworkTable contour, NetworkTable leftLine, NetworkTable rightLine) {
		contourInput = contour;
		leftLineInput = leftLine;
		rightLineInput = rightLine;
		cameraLocation = new Pose(CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_YAW, CAMERA_PITCH, CAMERA_ROLL);
		targetL = new Vector3d();
		targetR = new Vector3d();
		targetC = new Vector3d();
		targetCUltraSonic = new Vector2d();
		targetLine = new Vector3d();
	}

	/**
	 * @return distance the camera is from the target.
	 */
	private double getUltraDistance() {
		return Robot.ultra.pidGet();
	}

	/**
	 * Processes data from network tables to update the target location relative to
	 * the robot.
	 */
	public void processData() {
		NetworkTableInstance.getDefault().flush();
		double[] defaultValue = new double[0];
		double[] centerX = contourInput.getEntry("centerX").getDoubleArray(defaultValue);
		double[] centerY = contourInput.getEntry("centerY").getDoubleArray(defaultValue);
		double[] area = contourInput.getEntry("area").getDoubleArray(defaultValue);
		double[] width = contourInput.getEntry("width").getDoubleArray(defaultValue);
		double[] height = contourInput.getEntry("height").getDoubleArray(defaultValue);

		// Checks that the camera has actually found the target.
		if (centerX.length >= 2) {
			VisionTarget left, right;
			ArrayList<VisionTarget> targets = new ArrayList<>();
			for (int i = 0; i < centerX.length; i++) {
				targets.add(new VisionTarget(centerX[i], centerY[i], width[i], height[i], area[i]));
			}

			// Sort targets so leftmost is first.
			Collections.sort(targets);

			if (targets.size() == 2) {
				left = targets.get(0);
				right = targets.get(1);
			} else {
				/*
				 * // Get all the values of left side target lines. double[] x1Left =
				 * leftLineInput.getEntry("x1").getDoubleArray(defaultValue); double[] y1Left =
				 * leftLineInput.getEntry("y1").getDoubleArray(defaultValue); double[] x2Left =
				 * leftLineInput.getEntry("x2").getDoubleArray(defaultValue); double[] y2Left =
				 * leftLineInput.getEntry("y2").getDoubleArray(defaultValue);
				 * 
				 * // Get all the values of right side target lines. double[] x1Right =
				 * rightLineInput.getEntry("x1").getDoubleArray(defaultValue); double[] y1Right
				 * = rightLineInput.getEntry("y1").getDoubleArray(defaultValue); double[]
				 * x2Right = rightLineInput.getEntry("x2").getDoubleArray(defaultValue);
				 * double[] y2Right =
				 * rightLineInput.getEntry("y2").getDoubleArray(defaultValue);
				 * 
				 * if (!(x1Left.length == x2Left.length && x2Left.length == y1Left.length &&
				 * y1Left.length == y2Left.length && y2Left.length == x1Left.length)) {
				 * processData(); return; } if (!(x1Right.length == x2Right.length &&
				 * x2Right.length == y1Right.length && y1Right.length == y2Right.length &&
				 * y2Right.length == x1Right.length)) { processData(); return; }
				 * 
				 * // Create array of left and right lines. Line2D[] leftLines = new
				 * Line2D[x1Left.length]; Line2D[] rightLines = new Line2D[x1Right.length]; for
				 * (int i = 0; i < leftLines.length; i++) leftLines[i] = new
				 * Line2D.Double(x1Left[i], y1Left[i], x2Left[i], y2Left[i]); for (int i = 0; i
				 * < rightLines.length; i++) rightLines[i] = new Line2D.Double(x1Right[i],
				 * y1Right[i], x2Right[i], y2Right[i]);
				 * 
				 * int errors = 0; // Finds the type of each target using the lines. for
				 * (VisionTarget t : targets) { boolean l = false; boolean r = false; // Checks
				 * if there are left lines in the target. for (Line2D line : leftLines) { if
				 * (t.contains(line)) { l = true; break; } }
				 * 
				 * // Checks if there are right lines in the target. for (Line2D line :
				 * rightLines) {
				 * 
				 * System.out .println(line.getX1() + "," + line.getY1() + "\t" + line.getX2() +
				 * "," + line.getY2()); if (t.contains(line)) { r = true; break; } }
				 * 
				 * if (l && r) {
				 * System.out.println("ERROR target somehow is both left and right");
				 * t.setType(TargetType.kBOTH); } else if (l) t.setType(TargetType.kLEFT); else
				 * if (r) t.setType(TargetType.kRIGHT); else { errors++;
				 * System.out.println("WARNING " + errors +
				 * " target found that has no left or right lines"); }
				 * 
				 * System.out.println(t); }
				 * 
				 * int centerTID = 0; VisionTarget centerTarget = targets.get(0); VisionTarget
				 * t; for (int i = 1; i < targets.size(); i++) { t = targets.get(i); if
				 * (Math.abs(t.cX - cx) < Math.abs(centerTarget.cX - cx) && t.getType() !=
				 * TargetType.kNONE) { centerTID = i; centerTarget = t; } }
				 * 
				 * if (centerTarget.getType() == TargetType.kLEFT) { left = centerTarget; if
				 * (centerTID < targets.size()) while (centerTID < targets.size() - 1 &&
				 * targets.get(++centerTID).getType() == TargetType.kNONE) ; right =
				 * targets.get(centerTID); } else { right = centerTarget; if (centerTID > 0)
				 * while (targets.get(--centerTID).getType() == TargetType.kNONE && centerTID >=
				 * 0) ; left = targets.get(centerTID); }
				 */
				left = targets.get(0);
				right = targets.get(1);
			}
			left = left.undistort();
			right = right.undistort();

			System.out.println(left + "\n" + right);

			// Gives angle in radians relative to center y line, left is negative
			double yawLeft = Math.atan((left.cX - IMAGE_WIDTH / 2) / FOCAL_LENGTH_X);
			double yawRight = Math.atan((right.cX - IMAGE_WIDTH / 2) / FOCAL_LENGTH_X);
			// Gives angle in radians relative to center x line, down is negative
			double pitchLeft = Math.atan((left.cY - IMAGE_HEIGHT / 2) / FOCAL_LENGTH_Y);
			double pitchRight = Math.atan((right.cY - IMAGE_HEIGHT / 2) / FOCAL_LENGTH_Y);
			SmartDashboard.putNumber("Pleft", pitchLeft);
			SmartDashboard.putNumber("Pright", pitchRight);

			// Rotate target angles to robot's reference frame
			yawLeft = yawLeft + cameraLocation.yaw;
			yawRight = yawRight + cameraLocation.yaw;
			pitchLeft = pitchLeft + cameraLocation.pitch;
			pitchRight = pitchRight + cameraLocation.pitch;

			// Uses an ultrasonic sensor to estimate the x and y location of the target
			// relative to robot.
			double yawMiddle = (yawLeft + yawRight) / 2;
			targetCUltraSonic.x = getUltraDistance() * Math.tan(yawMiddle) + cameraLocation.x;
			targetCUltraSonic.y = getUltraDistance() + cameraLocation.y;

			

			// Using some trig (see MAGIC), converts the pitch and yaw to unit vectors in
			// the direction of the target for each side.
			targetL.set(Math.sin(yawLeft) * Math.cos(pitchLeft), Math.cos(yawLeft) * Math.cos(pitchLeft),
					Math.sin(pitchLeft));
			targetR.set(Math.sin(yawRight) * Math.cos(pitchRight), Math.cos(yawRight) * Math.cos(pitchRight),
					Math.sin(pitchRight));

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
			targetLine.set(targetR.x - targetL.x, targetR.y - targetL.y, 0);
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
		return Math.toDegrees(Math.atan(targetLine.y / targetLine.x));
	}

	/**
	 * @return Degrees needed to rotate so the robot is facing the target. CW is
	 *         positive.
	 */
	public double rawDegrees() {
		return Math.toDegrees(Math.atan(targetC.x / targetC.y));
	}

	/**
	 * @return Distance needed to be traveled in the x direction after the robot has
	 *         rotated.
	 */
	public double distanceX() {
		// Angle needed to be turned
		double yaw = Math.atan(targetLine.y / targetLine.x);
		// Rotation of axes
		double xPrimeC = targetC.x * Math.cos(yaw) + targetC.y * Math.sin(yaw);
		return xPrimeC;
	}

	/**
	 * Factor that normalizes the pixels by making the top left corner of the image
	 * 1 unit from the center.
	 */
	public final double NORMALIZATION_FACTOR = Math.sqrt(Math.pow(IMAGE_WIDTH / 2, 2) + Math.pow(IMAGE_HEIGHT / 2, 2));

	public Point2D undistort(double x, double y) {
		// Normalize pixel location.
		x = (x - PRINCIPAL_POINT_X) / (NORMALIZATION_FACTOR);
		y = (y - PRINCIPAL_POINT_Y) / (NORMALIZATION_FACTOR);

		// Apply undistortion
		double r = Math.pow(x, 2) + Math.pow(y, 2);
		double factor = (1 + k1 * r + k2 * Math.pow(r, 2));
		double xFixed = x * factor;
		double yFixed = y * factor;

		// Revert normalization back to original origin.
		xFixed = xFixed * NORMALIZATION_FACTOR + PRINCIPAL_POINT_X;
		yFixed = yFixed * NORMALIZATION_FACTOR + PRINCIPAL_POINT_Y;

		return new Point2D.Double(xFixed, yFixed);
	}

	/**
	 * @return Distance needed to be traveled in the y direction after the robot has
	 *         rotated.
	 */
	public double distanceY() {
		// Angle needed to be turned
		double yaw = Math.atan(targetLine.y / targetLine.x);
		// Rotation of axes()
		double yPrimeC = -targetC.x * Math.sin(yaw) + targetC.y * Math.cos(yaw);
		return yPrimeC;
	}

	/**
	 * @return Distance needed to be traveled in the y direction according to
	 *         ultrasonic sensor.
	 */
	public double ultraDistanceY() {
		return getUltraDistance();
	}

	// private final double xConversionFactor = 5.0 / (3.0 - .125);
	private final double xConversionFactor = 1;

	/**
	 * @return Distance needed to be traveled in the y direction according to
	 *         ultrasonic sensor and vision.
	 */
	public double ultraDistanceX() {
		return targetCUltraSonic.x * xConversionFactor;
	}

	/**
	 * Emprical value found by measuring the percent error when travelling fowards.
	 * 
	 * Example: Drive 36in and measure real value. a_x = real value/36in. Find the
	 * average over multiple trials will make it better.
	 */
	final double a_x = 1;

	/**
	 * Emprical value found by measuring the percent error when travelling sideways.
	 * 
	 * Example: Drive 36in and measure real value. a_r = real value/36in. Find the
	 * average over multiple trials will make it better.
	 */
	final double a_r = 1;

	/**
	 * Emprical value found by measuring the percent error when rotating.
	 * 
	 * Example: Rotate 720deg and measure real value. a_z = real value/720deg. Find
	 * the average over multiple trials will make it better.
	 */
	final double a_z = 1;

	/**
	 * Radius of mecanum wheels.
	 */
	final double MECANUM_RADIUS = 3;

	/**
	 * Distance of mecanum wheel from center of gravity in forwards backwards
	 * direction.
	 */
	final double Y_OFFSET = 29 / 2;

	/**
	 * Distance of mecanum wheel from center of gravity in forwards left right
	 * direction.
	 */
	final double X_OFFSET = 29 / 2;

	/**
	 * Gives required displacements of each wheel to reach desired target relative
	 * to robot.
	 * 
	 * Coordinate System: </br>
	 * Origin is center of robot. </br>
	 * y+ is forwards from the origin. </br>
	 * x+ is right from the origin. </br>
	 * CCW rotation is positive. </br>
	 * 
	 * @param x   x-coordinate relative to robot in inches.
	 * @param y   y-coordinate relative to robot in inches.
	 * @param rot angle to rotate relative to robot in degrees. -2pi<rot<2pi
	 * @return How many radians each wheel must rotate to reach desired location
	 */
	public double[] mecanumPath(double x, double y, double rot) {
		// Converts incoming coordinates to the coordinates needed in the math.
		// Coordinates in math come from this paper:
		// https://repository.tudelft.nl/islandora/object/uuid:6b6cd917-8a03-4448-903e-33d4428d3831/datastream/OBJ/download
		double temp = y;
		y = -x;
		x = temp;
		rot = -Math.toRadians(rot);

		// To avoid divide by 0 error (basically a limit)
		if (rot == 0)
			rot = 0.000001;

		// This is actually just magic don't look at it.
		double FL = -(rot * (a_z * y * Math.cos(rot) - 2 * Y_OFFSET * a_r - a_z * y - 2 * X_OFFSET * a_r
				+ a_z * y * Math.sin(rot) + a_r * a_z * x + 2 * X_OFFSET * a_r * Math.cos(rot)
				+ 2 * Y_OFFSET * a_r * Math.cos(rot) - a_r * a_z * x * Math.cos(rot) + a_r * a_z * x * Math.sin(rot)))
				/ (2 * MECANUM_RADIUS * a_x * a_r * a_z * (Math.cos(rot) - 1));

		double FR = -(rot * (2 * X_OFFSET * a_r + 2 * Y_OFFSET * a_r - a_z * y + a_z * y * Math.cos(rot)
				- a_z * y * Math.sin(rot) - a_r * a_z * x - 2 * X_OFFSET * a_r * Math.cos(rot)
				- 2 * Y_OFFSET * a_r * Math.cos(rot) + a_r * a_z * x * Math.cos(rot) + a_r * a_z * x * Math.sin(rot)))
				/ (2 * MECANUM_RADIUS * a_x * a_r * a_z * (Math.cos(rot) - 1));

		double RL = (rot * (2 * X_OFFSET * a_r + 2 * Y_OFFSET * a_r + a_z * y - a_z * y * Math.cos(rot)
				+ a_z * y * Math.sin(rot) + a_r * a_z * x - 2 * X_OFFSET * a_r * Math.cos(rot)
				- 2 * Y_OFFSET * a_r * Math.cos(rot) - a_r * a_z * x * Math.cos(rot) - a_r * a_z * x * Math.sin(rot)))
				/ (2 * MECANUM_RADIUS * a_x * a_r * a_z * (Math.cos(rot) - 1));

		double RR = -(rot * (2 * X_OFFSET * a_r + 2 * Y_OFFSET * a_r - a_z * y + a_z * y * Math.cos(rot)
				+ a_z * y * Math.sin(rot) + a_r * a_z * x - 2 * X_OFFSET * a_r * Math.cos(rot)
				- 2 * Y_OFFSET * a_r * Math.cos(rot) - a_r * a_z * x * Math.cos(rot) + a_r * a_z * x * Math.sin(rot)))
				/ (2 * MECANUM_RADIUS * a_x * a_r * a_z * (Math.cos(rot) - 1));

		double[] out = new double[4];
		out[0] = FL;
		out[1] = FR;
		out[2] = RL;
		out[3] = RR;
		return out;
	}
}
