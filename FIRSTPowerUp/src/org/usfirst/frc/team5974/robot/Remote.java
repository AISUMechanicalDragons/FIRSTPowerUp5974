package org.usfirst.frc.team5974.robot;

import org.usfirst.frc.team5974.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;

public class Remote {
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
	int dPad;					    //d-pad
	boolean joystickLPress;		    //left joystick button press
	boolean joystickRPress;		    //right joystick button press
	boolean buttonStart;			//start button
	boolean buttonBack;			    //back button
	
	/**Button ports, however, do start at 1 with one being the 'A' button. - Thomas*/
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
	
	boolean tankDriveBool = true;	//drive mode: true = tank drive, false = arcade drive
	boolean fastBool = false;		//speed mode: true = fast mode, false = slow mode
	boolean grabberInBool = true;	//grabber: true = in, false = out
	
	
	public boolean checkButton(boolean button, boolean toggle, int port) {		//When the button is pushed, once it is released, its toggle is changed
		if (button) {
			toggle = !toggle;
			while (button) {		//TODO while loop causes problems
				button = controller.getRawButton(port);
			}
		}
		return toggle;
	}
	
	public void joystickDeadZone() {		//sets dead zone for joysticks		//TODO test this
		if (joystickLXAxis <= 0.075 && joystickLXAxis >= -0.075) {
			joystickLXAxis = 0;
		} if (joystickLYAxis <= 0.075 && joystickLYAxis >= -0.075) {
			joystickLYAxis = 0;
		}
		if (joystickRXAxis <= 0.075 && joystickRXAxis >= -0.075) {
			joystickRXAxis = 0;
		} if (joystickRYAxis <= 0.075 && joystickRYAxis >= -0.075) {
			joystickRYAxis = 0;
		}
	}
	
	public void updateController() {		//updates all controller features
		/**I don't know why you made a whole bunch of port variables when numbers are faster, but hey! You do you. - Thomas*/
		//i concur --Carter
		
		//left joystick update
		joystickLXAxis = controller.getRawAxis(portJoystickLXAxis);		//returns a value [-1,1]
		joystickLYAxis = controller.getRawAxis(portJoystickLYAxis);		//returns a value [-1,1]
		joystickLPress = controller.getRawButton(portJoystickLPress);	//returns a value {0,1}
		
		//right joystick update
		joystickRXAxis = controller.getRawAxis(portJoystickRXAxis);		//returns a value [-1,1]
		joystickRYAxis = controller.getRawAxis(portJoystickRYAxis);		//returns a value [-1,1]
		joystickRPress = controller.getRawButton(portJoystickRPress);	//returns a value {0,1}
		
		//trigger updates
		triggerL = controller.getRawAxis(portTriggerL);		//returns a value [0,1]
		triggerR = controller.getRawAxis(portTriggerR);		//returns a value [0,1]
		
		//bumper updates
		bumperL = controller.getRawButton(portBumperL);		//returns a value {0,1}
		bumperR = controller.getRawButton(portBumperR);		//returns a value {0,1}
		
		//button updates
		buttonX = controller.getRawButton(portButtonX);		//returns a value {0,1}
		buttonY = controller.getRawButton(portButtonY);		//returns a value {0,1}
		buttonA = controller.getRawButton(portButtonA);		//returns a value {0,1}
		buttonB = controller.getRawButton(portButtonB);		//returns a value {0,1}
		
		buttonBack = controller.getRawButton(portButtonBack);	//returns a value {0,1}
		buttonStart = controller.getRawButton(portButtonStart);	//returns a value {0,1}
		
		//toggle checks
		tankDriveBool = checkButton(buttonX, tankDriveBool, portButtonX);		//toggles boolean if button is pressed
		fastBool = checkButton(buttonB, fastBool, portButtonB);					//toggles boolean if button is pressed
		
		
		//d-pad/POV updates
		dPad = controller.getPOV(portDPad);		//returns a value {-1,0,45,90,135,180,225,270,315}

		//d-pad/POV turns
		if (dPad != -1) {
			dPad = 360 - dPad; //Converts the clockwise dPad rotation into a Gyro-readable counterclockwise rotation.
			rotateTo(dPad);
		}
		
		joystickDeadZone();
	}
}
