/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.vision;

import java.awt.Rectangle;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

/**
 * Holds a target found by vision processing.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class VisionTarget implements Comparable<VisionTarget> {
	public double cX, cY, area, w, h;
	Rectangle2D bound;
	TargetType type = TargetType.kNONE;

	public VisionTarget(double centerX, double centerY, double width, double height, double area) {
		cX = centerX;
		cY = centerY;
		w = width;
		h = height;
		this.area = area;
		bound = new Rectangle2D.Double(cX - w / 2.0, cY - h / 2.0, w, h);
	}

	public void setType(TargetType t) {
		type = t;
	}

	public TargetType getType() {
		return type;
	}

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

	public boolean contains(Line2D line) {
		return bound.intersectsLine(line);
	}
}
