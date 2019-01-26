/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.vision;

import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

import frc.robot.utils.VisionCargoBay;

/**
 * Holds a target found by vision processing.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class VisionTarget implements Comparable<VisionTarget> {
	public double cX, cY, area, w, h;
	Rectangle2D bound;
	TargetType type = TargetType.kNONE;

	/**
	 * Holds a contour target found by the camera.
	 * 
	 * @param centerX
	 * @param centerY
	 * @param width
	 * @param height
	 * @param area
	 */
	public VisionTarget(double centerX, double centerY, double width, double height, double area) {
		cX = centerX;
		cY = centerY;
		w = width;
		h = height;
		this.area = area;
		bound = new Rectangle2D.Double(cX - w / 2.0, cY - h / 2.0, w, h);
	}

	/**
	 * @param t Type to set the target to.
	 * @see VisionTarget.TargetType
	 */
	public void setType(TargetType t) {
		type = t;
	}

	/**
	 * @return Type of target.
	 * @see VisionTarget.TargetType
	 */
	public TargetType getType() {
		return type;
	}

	/**
	 * The different target types:
	 * 
	 * kLEFT means it should be on left side of bay. <br>
	 * kRIGHT means it should be on right side of bay. <br>
	 * kNONE means it can't be determined what side it should be on. <br>
	 */
	public enum TargetType {
		kLEFT, kRIGHT, kNONE;
	}

	@Override
	public int compareTo(VisionTarget other) {
		// compareTo should return < 0 if this is supposed to be
		// less than other, > 0 if this is supposed to be greater than
		// other and 0 if they are supposed to be equal
		return (int) (this.cX - other.cX);
	}

	/**
	 * Check if a line segment is within the target.
	 * 
	 * @param line A line segment in 2D space.
	 * @return true if any part of the line segment is in the target.
	 */
	public boolean contains(Line2D line) {
		return bound.intersectsLine(line);
	}
}
