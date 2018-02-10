package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.command.Command;


public class leftAuto extends Command {
	
	Robot robot = new Robot();
	public leftAuto() {
		
	}
	public void initialize() {
		
	}
	protected void execute() {	
		if (robot.gameData.charAt(0) == 'L'){
			//move forward
			robot.moveDistance(2.5146,0);
			//turn right
			robot.moveDistance(1.4732, 270);
			//turn left
			robot.moveDistance(0.254, 0);
		}
		else {
		//move forward
		robot.moveDistance(4.3,0);
		// TODO Auto-generated constructor stub
		}
	}
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return true;
	}

}
