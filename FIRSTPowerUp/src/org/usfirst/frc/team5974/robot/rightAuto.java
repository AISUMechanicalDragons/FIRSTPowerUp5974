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


public class rightAuto extends Command {

	public rightAuto() {
		if (gameData.charAt(0)== 'L') {
			moveDistance(4.3, 0);		//move forward 4.3 m
		}
		else {
			moveDistance(4.3, 0);		//move forward 4.3 m
			moveDistance(0.78, 90);			//rotate towards switch and move .78 m towards it
		}
		// TODO Auto-generated constructor stub
	}

}
