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

public class middleAuto extends Robot {

	public middleAuto() {
		if (gameData.charAt(0) == 'L') {
			//Move forward
			moveDistance(1.524,0);
			//Turn to 90(left) degrees and move
			moveDistance(2.413,90);
			//Turn to 0 Degrees
			moveDistance(1.778, 0);
		}
		else {
			moveDistance(1.63, 0);			//move forward 1.63 m
			moveDistance(0.91, 315);			//move at a 315 degree for 1.63 m
			moveDistance(0.76, 0);
		}
		// TODO Auto-generated constructor stub
	}

}
