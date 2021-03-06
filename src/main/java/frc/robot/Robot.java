/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CameraTurn;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleOpDrive;
import frc.robot.iodevices.AnalogUltraSonic;
import frc.robot.iodevices.oled.OLEDDisplay;
import frc.robot.subsystems.CargoHold;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Ejector;
import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.HPClaw;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wintch;
import frc.robot.utils.TalonConfig;
//import frc.robot.utils.VisionCargoBay;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * @author FRC Team 3389 TEC Tigers
 */
public class Robot extends TimedRobot {
	public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
	public static OI operatorControllers;
	public static TalonConfig talonConfig = new TalonConfig();

	public static final DriveTrain drive_Train = new DriveTrain();

	public static final Intake intake = new Intake();
	public static final Lift lift = new Lift();
	public static final Wintch wintch = new Wintch();
	public static final Ejector eject = new Ejector();
	public static final CargoHold cargo = new CargoHold();

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	public static NetworkTable lines;

	public static NetworkTable bayReport;
	public static NetworkTable leftLineReport;
	public static NetworkTable rightLineReport;
	// public static VisionCargoBay bay;
	public static int bayListener;

	public static AnalogUltraSonic ultra = new AnalogUltraSonic(RobotMap.ULTRA_INPUT);

	public static OLEDDisplay robotScreen;

	UsbCamera camera;

	public static Preferences prefs;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		operatorControllers = new OI();
		m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
		m_chooser.addOption("Camera Tester", new CameraTurn(drive_Train.getGyro(), 0));
		SmartDashboard.putData("Auto mode", m_chooser);

		if (RobotMap.CONFIG_TALONS) {
			drive_Train.configTalons(talonConfig.talon);
			lift.configTalons(talonConfig.talon);
			intake.configTalons(talonConfig.talon);
		}

		camera = CameraServer.getInstance().startAutomaticCapture();

		// Starts streaming camera to driver station and gets results from GRIP
		// camera = CameraServer.getInstance().startAutomaticCapture();
		// camera.setResolution(720, 480);
		// camera.setBrightness(0);
		// camera.setExposureManual(20);
		// camera.setFPS(10);
		// camera.setWhiteBalanceManual(0);

		lines = NetworkTableInstance.getDefault().getTable("GRIP/lineReport");

		bayReport = NetworkTableInstance.getDefault().getTable("GRIP/bayReport");
		leftLineReport = NetworkTableInstance.getDefault().getTable("GRIP/leftLineReport");
		rightLineReport = NetworkTableInstance.getDefault().getTable("GRIP/rightLineReport");
		// bay = new VisionCargoBay(bayReport, leftLineReport, rightLineReport);

		robotScreen = new OLEDDisplay();

		prefs = Preferences.getInstance();

		// bayListener = bayReport.addEntryListener("height", new TableEntryListener() {
		// @Override
		// public void valueChanged(NetworkTable table, String key, NetworkTableEntry
		// entry, NetworkTableValue value,
		// int flags) {
		// bay.processData();
		// }
		// }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();
		Scheduler.getInstance().add(new TeleOpDrive());

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		Scheduler.getInstance().add(new TeleOpDrive());
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		// SmartDashboard.putNumber("Raw Degrees", bay.rawDegrees());//
		// SmartDashboard.putNumber("Yaw Degrees", bay.yawDegrees());

		// SmartDashboard.putNumber("Ultra Distance Y", bay.ultraDistanceY()); // Y
		// distance away from target based off of
		// // the ultradistance sensor
		// SmartDashboard.putNumber("Ultra Distance X", bay.ultraDistanceX()); // X
		// distance away from target based off of
		// // the ultradistance sensor
		// SmartDashboard.putBoolean("TargetID", bay.getTargetID()); // return true if
		// cargo bay's reflective tape is in
		// // view
		// SmartDashboard.putNumber("Delta X", bay.deltaX()); // X distance away from
		// cargo bay based off of trig
		// // calculations
		// SmartDashboard.putNumber("Delta Y", bay.deltaY()); // Y distance away from
		// cargo bay based off of trig
		// // calculations

		// SmartDashboard.putNumber("ClawL Position", claw.getLAngle());
		// SmartDashboard.putNumber("ClawR Position", claw.getRAngle());
		// SmartDashboard.putNumber("FL Displacement", bay.mecanumPath(bay.deltaX(),
		// bay.deltaY(), bay.yawDegrees())[0]);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
}
