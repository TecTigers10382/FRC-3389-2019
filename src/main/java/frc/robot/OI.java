/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.TeleOpEjector;
import frc.robot.commands.TeleOpHP;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class OI {
	Joystick jsDriver = new Joystick(0);
	Joystick jsOperator = new Joystick(1);

	Button claw = new JoystickButton(jsOperator, 5);//remember to change later
	Button align = new JoystickButton(jsDriver, 8);//remember to change later
	Button eject = new JoystickButton(jsOperator, 9);//remember to change later

	/**
	 * Sets up claw as toggle button to control the hatch panel claw.
	 */
	public OI() {

		claw.toggleWhenPressed(new TeleOpHP());
		eject.toggleWhenPressed(new TeleOpEjector());
		align.whenPressed(new AutoAlign());
	}

	/**
	 * Let's other objects get values from the left joystick.
	 * 
	 * @return Returns the leftStick object
	 */
	public Joystick getDriverJoystick() {
		return jsDriver;
	}

	/**
	 * Let's other objects get values from the right joystick.
	 * 
	 * @return Returns the rightStick object
	 */
	public Joystick getOperatorJoystick() {
		return jsOperator;
	}

	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
