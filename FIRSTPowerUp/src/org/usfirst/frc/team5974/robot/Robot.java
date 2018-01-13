/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
Controls by Action:

	Arcade Drive:
		-Left joystick:
			Rotate
			Forward
			Backward
		
	Tank Drive:
		-Left joystick:
			Left wheels
				Front
				Back
		-Right joystick
			Right wheels
				Front
				Back
				
	Toggle Speed:
		-B button
	
	Toggle Drive Style:
		-X button
				
	Climb:
		-Left trigger:
			Down
		-Right trigger:
			Up
		
	Grabber Wheels:
		-Left bumper:
			Spin out
		-Right bumper:
			Spin in
*/

/*
 Controls by Buttons:
 
 	Left Trigger: Climb down
 	Right Trigger: Climb up
 	
 	Left Bumper: Grabber spin out
 	Right Bumper: Grabber spin in
 	
 	Left Joystick: Tank/arcade drive
 	Right Joystick: Tank drive
 	
 	D-Pad: None
 	
 	A Button: None
 	B Button: Toggle speed
 	X Button: Toggle drive style
 	Y Button: None
 	
 	Back Button: None
 	Select Button: None
 */

package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //Dashboard
import edu.wpi.first.wpilibj.Joystick; //Controller
import edu.wpi.first.wpilibj.Timer; //Timer
import edu.wpi.first.wpilibj.Spark; //Motor Controller
import edu.wpi.first.wpilibj.*; //everything tbh
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
/*
 
 */

public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default"; //any idea what these are for??
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	//Motors. We NEED to change these to the actual motors, once electrical makes up its mind.
	//Also, sometimes one side is inverted. If it is, we need to change our drive code to reflect that.
	Spark motorRB = new Spark(1); //motor right back
	Spark motorRF = new Spark(2); //motor right front
	Spark motorLB = new Spark(3); //motor left back
	Spark motorLF = new Spark(4); //motor left front
	
	//Variables we're using
	Joystick controller = new Joystick(0);			//controller
	double joystickLXAxis;			//left joystick x-axis
	double joystickLYAxis;			//left joystick y-axis
	double joystickRXAxis;			//right joystick x-axis
	double joystickRYAxis;			//right joystick y-axis
	double triggerL;				//left trigger
	double triggerR;				//right trigger
	boolean bumperL;				//left bumper
	boolean bumperR;				//right bumper
	boolean buttonX;				//x button
	boolean buttonY;				//y button
	boolean buttonA;				//a button
	boolean buttonB;				//b button
	double dPad;					//d-pad
	
	double robotSpeed;			//robot speed (fast/slow mode)
	boolean tankDriveBool = true;		//tank drive boolean: true = tank drive, false = arcade drive
	boolean fastBool = false;			//fast boolean: true = fast mode, false = slow mode
	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		update();
		dashboardOutput();
		
		if (tankDriveBool) {
			tankDrive();
		} else {
			arcadeDrive();
		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void dashboardOutput() {	//sends data to dashboard and displays it on dashboard
		
	}
	
	public void tankDrive() {	//tank drive: left joystick controls left wheels, right joystick controls right wheels
		//right motors = right joystick y-axis
		//left motors = left joystick y-axis
		if (fastBool) {
			motorRB.set(joystickRYAxis);
			motorRF.set(joystickRYAxis);
			motorLB.set(joystickLYAxis);
			motorLF.set(joystickLYAxis);
		} else {
			motorRB.set(joystickRYAxis/2);
			motorRF.set(joystickRYAxis/2);
			motorLB.set(joystickLYAxis/2);
			motorLF.set(joystickLYAxis/2);
		}
	}
	
	public void arcadeDrive() {	//arcade drive: left joystick controls all driving
		//right wheels have less power the farther right the left joystick is and more power the farther left
		//left wheels have less power the farther left the left joystick is and more power the farther right
		if (fastBool) {
			motorRB.set(joystickLYAxis - joystickLXAxis);
			motorRF.set(joystickLYAxis - joystickLXAxis);
			motorLB.set(joystickLYAxis + joystickLXAxis);
			motorLF.set(joystickLYAxis + joystickLXAxis);
		} else {
			motorRB.set((joystickLYAxis - joystickLXAxis)/2);
			motorRF.set((joystickLYAxis - joystickLXAxis)/2);
			motorLB.set((joystickLYAxis + joystickLXAxis)/2);
			motorLF.set((joystickLYAxis + joystickLXAxis)/2);
		}
	}
	
	public void update() {
		//left joystick update
		joystickLXAxis = controller.getRawAxis(0);
		joystickLYAxis = controller.getRawAxis(1);
		
		//right joystick update
		joystickRXAxis = controller.getRawAxis(4);
		joystickRYAxis = controller.getRawAxis(5);
		
		//trigger updates
		triggerL = controller.getRawAxis(2);
		triggerR = controller.getRawAxis(3);
		
		//bumper updates
		bumperL = controller.getRawButton(5);
		bumperR = controller.getRawButton(6);
		
		//button updates
		//buttonX = controller.getRawButton();
		//buttonY = controller.getRawButton();
		//buttonA = controller.getRawButton();
		//buttonB = controller.getRawButton();
	}
	
	public void joystickDeadZone() { //dead zone for joysticks
		if (joystickLXAxis <= 0.15 && joystickLXAxis <= -0.15) {
			joystickLXAxis = 0;
		} if (joystickLYAxis <= 0.15 && joystickLYAxis <= -0.15) {
			joystickLYAxis = 0;
		}
	}
}
