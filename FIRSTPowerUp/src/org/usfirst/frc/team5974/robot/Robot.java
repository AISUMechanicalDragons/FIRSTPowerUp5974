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
	
	Toggle Drive Style: (arcade/tank)
		-X button
				
	Toggle Grabber In/Out:
		-Y button
	
	Climb:
		-Left trigger:
			Down
		-Right trigger:
			Up
		
	Grabber Wheels:
		-Left bumper:
			Spin left side
		-Right bumper:
			Spin right side
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
 	Y Button: Toggle grabber in/out
 	
 	Back Button: None
 	Select Button: None
 */

package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;		//Dashboard
import edu.wpi.first.wpilibj.Joystick;		//Controller
import edu.wpi.first.wpilibj.Timer;		//Timer
import edu.wpi.first.wpilibj.Spark;		//Motor Controller
import edu.wpi.first.wpilibj.*;		//everything tbh
import org.usfirst.frc.team5974.robot.ADIS16448_IMU;		//IMU
//import java.util.ArrayList;		//arraylist

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
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
	Joystick controller = new Joystick(0);			//controller  // could be incorrect port
	ADIS16448_IMU IMU = new ADIS16448_IMU();		//imu: accelerometer and gyro
	
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
	int dPad;					//d-pad
	boolean joystickLPress;		//left joystick button press
	boolean joystickRPress;		//right joystick button press
	boolean buttonStart;			//start button
	boolean buttonBack;			//back button
	
	
	int portButtonX = 3;
	int portButtonY = 4;
	int portButtonA = 1;
	int portButtonB = 2;
	
	int portJoystickLPress = 9;
	int portJoystickRPress = 10;
	
	int portJoystickLXAxis = 0;
	int portJoystickLYAxis = 1;
	int portJoystickRXAxis = 4;
	int portJoystickRYAxis = 5;
	
	int portTriggerL = 2;
	int portTriggerR = 3;
	
	int portBumperL = 5;
	int portBumperR = 6;
	
	int portButtonBack = 7;
	int portButtonStart = 8;
	
	int portDPad = 0;
	
	double angleToForward = 0;
	
	double robotSpeed;			//robot speed (fast/slow mode)
	boolean tankDriveBool = true;		//tank drive boolean: true = tank drive, false = arcade drive
	boolean fastBool = false;			//fast boolean: true = fast mode, false = slow mode
	boolean grabberInBool = true;			//true = in, false = out
	
	//position arrays
	double posX = 0;
	double posY = 0;
	double posZ = 0;
	
	//velocity arrays
	double velX = 0;
	double velY = 0;
	double velZ = 0;
	
	//acceleration arrays
	double accelX = 0;
	double accelY = 0;
	double accelZ = 0;
	
	//time variables
	Timer timer = new Timer();
	double dT = 0;
	double t0 = 0;
	double t1 = 0;
	
	//this is the variable that gives us switch and scale sides in format LRL or RRL, etc
	String gameData;
	
	public boolean checkButton(int port, boolean toggle) {		//If the button is pushed, once it is released, its toggle is changed
		if (controller.getRawButton(port)) {
			toggle = !toggle;
			while (controller.getRawButton(port)) {}
		}
		return toggle;
	}
	
	public void rotateTo(int goTo) {		//rotates robot to angle based on IMU and d-pad
		//clockwise degrees to goTo angle
		double cw = (goTo - angleToForward < 0) ? (goTo - angleToForward + 360) : (goTo - angleToForward);
		
		//counter-clockwise degrees to goTo angle
		double ccw = (angleToForward - goTo< 0) ? (angleToForward - goTo + 360) : (angleToForward - goTo);
		
		//rotates the fastest way until in +- 5 of goTo angle
		while (goTo >= angleToForward + 5 && goTo <= angleToForward - 5) {
			updateGyro();
			if (cw <= ccw) {
				motorRB.set(-1);
				motorRF.set(-1);
				motorLB.set(1);
				motorLF.set(1);
			} else {
				motorRB.set(1);
				motorRF.set(1);
				motorLB.set(-1);
				motorLF.set(-1);
			}
		}
	}
	
	public double withIn(double input, double upperBound, double lowerBound) {		//returns the inputed value if inside the bounds. returns the bound it is past if it is past a bound.
		if (input > 0) {
			return java.lang.Math.min(upperBound, input);
		} else if (input < 0) {
			return java.lang.Math.max(lowerBound, input);
		} else {
			return 0;
		}
	}
	
	public void joystickDeadZone() {		//sets dead zone for joysticks
		if (joystickLXAxis <= 0.1 && joystickLXAxis <= -0.1) {
			joystickLXAxis = 0;
		} if (joystickLYAxis <= 0.1 && joystickLYAxis <= -0.1) {
			joystickLYAxis = 0;
		}
	}
	
	public void updateGyro() {		//set IMU.getAngle() (-inf,inf) output to a non-looping value [0,360)
		angleToForward = IMU.getAngle();
		if (angleToForward >= 360) {
			angleToForward -= 360;
		} else if (angleToForward < 0) {
			angleToForward += 360;
		}
	}
	
	public void updateTimer() {		//sets change in time between the current running of a periodic function and the previous running
		t0 = t1;
		t1 = timer.get();
		dT = t1 - t0;
	}
	
	public void updateTrifecta() {	//updates pos, vel, and accel
		//accel variables updated from IMU
		accelX = IMU.getAccelX();
		accelY = IMU.getAccelY();
		accelZ = IMU.getAccelZ();
		
		//vel updated by integral of accel
		velX += accelX * dT;
		velY += accelY * dT;
		velZ += accelZ * dT;
		
		//pos updated by integral of vel and adjusted for robot rotation
		posX += velX * dT * Math.sin(angleToForward * (Math.PI / 180.0));
		posY += velY * dT * Math.cos(angleToForward * (Math.PI / 180.0));
		posZ += velZ * dT;
	}
	
	public void updateController() {		//updates all controller features
		//left joystick update
		joystickLXAxis = controller.getRawAxis(portJoystickLXAxis);
		joystickLYAxis = controller.getRawAxis(portJoystickLYAxis);
		joystickLPress = controller.getRawButton(portJoystickLPress);
		
		//right joystick update
		joystickRXAxis = controller.getRawAxis(portJoystickRXAxis);
		joystickRYAxis = controller.getRawAxis(portJoystickRYAxis);
		joystickRPress = controller.getRawButton(portJoystickRPress);
		
		//trigger updates
		triggerL = controller.getRawAxis(portTriggerL);
		triggerR = controller.getRawAxis(portTriggerR);
		
		//bumper updates
		bumperL = controller.getRawButton(portBumperL);
		bumperR = controller.getRawButton(portBumperR);
		
		//button updates
		buttonX = controller.getRawButton(portButtonX);
		buttonY = controller.getRawButton(portButtonY);
		buttonA = controller.getRawButton(portButtonA);
		buttonB = controller.getRawButton(portButtonB);
		
		buttonBack = controller.getRawButton(portButtonBack);
		buttonStart = controller.getRawButton(portButtonStart);
		
		//toggle checks
		tankDriveBool = checkButton(portButtonX, tankDriveBool);
		fastBool = checkButton(portButtonB,fastBool);
		grabberInBool = checkButton(portButtonY, grabberInBool);
		
		//d-pad/POV updates
		dPad = controller.getPOV(portDPad);

		//d-pad/POV turns
		if (dPad != -1) {
			rotateTo(dPad);
		}
		
		joystickDeadZone();
	}
	
	public void update() {	//updates all update functions
		updateController();
		updateTimer();
		updateTrifecta();
		updateGyro();
	}
	
	public void dashboardOutput() {	//sends data to dashboard and displays it on dashboard
		SmartDashboard.putNumber("x-position", posX);
		SmartDashboard.putNumber("y-position", posY);
		SmartDashboard.putNumber("Speed", velY);
		SmartDashboard.putNumber("Angle to Forwards", angleToForward);
		SmartDashboard.putBoolean("Tank Drive Style", tankDriveBool);
		SmartDashboard.putBoolean("Fast Mode", fastBool);
		SmartDashboard.putBoolean("Grabber In", grabberInBool);
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
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		IMU.calibrate();
		IMU.reset();
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
		gameData = DriverStation.getInstance().getGameSpecificMessage(); 
		/*Gives 3 characters telling your switch and scale sides.
		 *The first one is your switch.
		 *The second is the scale.
		 *The third one is your opponent's switch
		*/
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
		/*To use gameData,example
		 * if(gameData.charAt(0) == 'L')     //if switch is on left side at character 0 (The first character)
		 * { //blah blahblah what to do if switch is on left yeah
		 * }
		 * else{
		 * 		//what to do if switch is on right.
		 * }
		 */
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		update();
		
		//dashboard outputs
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
}