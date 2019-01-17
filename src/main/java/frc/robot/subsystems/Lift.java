/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
    TalonSRX liftR, liftL;

    public Lift() {
        liftL = new TalonSRX(RobotMap.LIFT_LEFT);
        liftR = new TalonSRX(RobotMap.LIFT_RIGHT);

        liftL.follow(liftR);

        liftR.setInverted(false);
        liftL.setInverted(InvertType.OpposeMaster);
    }

    public void rawLift(double power) {
        liftR.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
