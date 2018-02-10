package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.command.Command;

public class middleAuto extends Command {
	
	Robot robot = new Robot();

	public middleAuto() {
		
	}
	
	public void initialize() {
		
	}
	
	protected void execute() {
		if (robot.gameData.charAt(0) == 'L') {
			//Move forward
			robot.moveDistance(1.524,0);
			//Turn to 90(left) degrees and move
			robot.moveDistance(2.413,90);
			//Turn to 0 Degrees
			robot.moveDistance(1.778, 0);
		}
		else {
			robot.moveDistance(1.63, 0);			//move forward 1.63 m
			robot.moveDistance(0.91, 315);			//move at a 315 degree for 1.63 m
			robot.moveDistance(0.76, 0);
		}
		// TODO Auto-generated constructor stub
	
	}
		
	protected boolean isFinished() {
		return true;
	}
	

}
