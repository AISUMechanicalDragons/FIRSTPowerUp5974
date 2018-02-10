package org.usfirst.frc.team5974.robot;

//import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;		//Dashboard
//import edu.wpi.first.wpilibj.Joystick;							//Controller
//import edu.wpi.first.wpilibj.Timer;								//Timer
//import edu.wpi.first.wpilibj.Spark;								//Motor Controller
//import edu.wpi.first.wpilibj.VictorSP;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.*;									//everything tbh
import org.usfirst.frc.team5974.robot.ADIS16448_IMU;			//IMU
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import java.util.ArrayList;		//arraylist


public class leftAuto extends Robot {

	public leftAuto() {
		if (gameData.charAt(0) == 'L'){
			//move forward
			moveDistance(2.5146,0);
			//turn right
			moveDistance(1.4732, 270);
			//turn left
			moveDistance(0.254, 0);
		}
		else {
		//move forward
		moveDistance(4.3,0);
		// TODO Auto-generated constructor stub
		}
	}

}
