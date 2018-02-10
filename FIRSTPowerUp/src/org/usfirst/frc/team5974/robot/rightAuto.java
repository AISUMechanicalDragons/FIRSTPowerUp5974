package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.command.Command;


public class rightAuto extends Command {

	Robot robot = new Robot();
	
	public rightAuto() {
		//super("rightAuto");
	}
	
	public void initialize() {

	}
	
	protected void execute() {
		if (robot.gameData.charAt(0)== 'L') {
			robot.moveDistance(4.3, 0);		//move forward 4.3 m
		}
		else {
			robot.moveDistance(4.3, 0);		//move forward 4.3 m
			robot.moveDistance(0.78, 90);			//rotate towards switch and move .78 m towards it
		}
		// TODO Auto-generated constructor stub
	}

	@Override
	protected boolean isFinished() {
		return true;
	}

}
