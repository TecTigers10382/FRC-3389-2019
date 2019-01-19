/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/**
 * Creates default talon config.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class TalonConfig {
	public TalonSRXConfiguration talon;

	public TalonConfig() {
		talon = new TalonSRXConfiguration();

		// For now it's default, change these when needed.
		// talon.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
		// talon.primaryPID.selectedFeedbackCoefficient = 0.328293;
		// talon.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
		// talon.auxiliaryPID.selectedFeedbackCoefficient = 0.877686;
		// talon.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
		// talon.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
		// talon.sum0Term = FeedbackDevice.QuadEncoder;
		// talon.sum1Term = FeedbackDevice.RemoteSensor0;
		// talon.diff0Term = FeedbackDevice.RemoteSensor1;
		// talon.diff1Term = FeedbackDevice.PulseWidthEncodedPosition;
		// talon.peakCurrentLimit = 20;
		// talon.peakCurrentDuration = 200;
		// talon.continuousCurrentLimit = 30;
		// talon.openloopRamp = 1.023000;
		// talon.closedloopRamp = 1.705000;
		// talon.peakOutputForward = 0.939394;
		// talon.peakOutputReverse = -0.289345;
		// talon.nominalOutputForward = 0.739980;
		// talon.nominalOutputReverse = -0.119257;
		// talon.neutralDeadband = 0.199413;
		// talon.voltageCompSaturation = 9.296875;
		// talon.voltageMeasurementFilter = 16;
		// talon.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
		// talon.velocityMeasurementWindow = 8;
		// talon.forwardLimitSwitchDeviceID = 6;
		// talon.reverseLimitSwitchDeviceID = 5;
		// talon.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
		// talon.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
		// talon.forwardSoftLimitThreshold = 2767;
		// talon.reverseSoftLimitThreshold = -1219;
		// talon.forwardSoftLimitEnable = true;
		// talon.reverseSoftLimitEnable = true;
		// talon.slot0.kP = 504.000000;
		// talon.slot0.kI = 5.600000;
		// talon.slot0.kD = 0.200000;
		// talon.slot0.kF = 19.300000;
		// talon.slot0.integralZone = 900;
		// talon.slot0.allowableClosedloopError = 217;
		// talon.slot0.maxIntegralAccumulator = 254.000000;
		// talon.slot0.closedLoopPeakOutput = 0.869990;
		// talon.slot0.closedLoopPeriod = 33;
		// talon.slot1.kP = 155.600000;
		// talon.slot1.kI = 5.560000;
		// talon.slot1.kD = 8.868600;
		// talon.slot1.kF = 454.000000;
		// talon.slot1.integralZone = 100;
		// talon.slot1.allowableClosedloopError = 200;
		// talon.slot1.maxIntegralAccumulator = 91.000000;
		// talon.slot1.closedLoopPeakOutput = 0.199413;
		// talon.slot1.closedLoopPeriod = 34;
		// talon.slot2.kP = 223.232000;
		// talon.slot2.kI = 34.000000;
		// talon.slot2.kD = 67.000000;
		// talon.slot2.kF = 6.323232;
		// talon.slot2.integralZone = 44;
		// talon.slot2.allowableClosedloopError = 343;
		// talon.slot2.maxIntegralAccumulator = 334.000000;
		// talon.slot2.closedLoopPeakOutput = 0.399804;
		// talon.slot2.closedLoopPeriod = 14;
		// talon.slot3.kP = 34.000000;
		// talon.slot3.kI = 32.000000;
		// talon.slot3.kD = 436.000000;
		// talon.slot3.kF = 0.343430;
		// talon.slot3.integralZone = 2323;
		// talon.slot3.allowableClosedloopError = 543;
		// talon.slot3.maxIntegralAccumulator = 687.000000;
		// talon.slot3.closedLoopPeakOutput = 0.129032;
		// talon.slot3.closedLoopPeriod = 12;
		// talon.auxPIDPolarity = true;
		// talon.remoteFilter0.remoteSensorDeviceID = 22;
		// talon.remoteFilter0.remoteSensorSource =
		// RemoteSensorSource.GadgeteerPigeon_Roll;
		// talon.remoteFilter1.remoteSensorDeviceID = 41;
		// talon.remoteFilter1.remoteSensorSource =
		// RemoteSensorSource.GadgeteerPigeon_Yaw;
		// talon.motionCruiseVelocity = 37;
		// talon.motionAcceleration = 3;
		// talon.motionProfileTrajectoryPeriod = 11;
		// talon.feedbackNotContinuous = true;
		// talon.remoteSensorClosedLoopDisableNeutralOnLOS = false;
		// talon.clearPositionOnLimitF = true;
		// talon.clearPositionOnLimitR = true;
		// talon.clearPositionOnQuadIdx = false;
		// talon.limitSwitchDisableNeutralOnLOS = true;
		// talon.softLimitDisableNeutralOnLOS = false;
		// talon.pulseWidthPeriod_EdgesPerRot = 9;
		// talon.pulseWidthPeriod_FilterWindowSz = 32;
		// talon.customParam0 = 3;
		// talon.customParam1 = 5;
	}
}
