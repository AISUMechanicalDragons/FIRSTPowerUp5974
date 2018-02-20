package org.usfirst.frc.team5974.robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class segwayBalance  extends PIDSubsystem{
	
	double inputAngle;
	public double motorMultiplier;
	
	
	public segwayBalance() {
		super("segwayBalance", 2.0, 0.0, 0.0);// The constructor passes a name for the subsystem and the P, I and D constants that are used when computing the motor output
		setAbsoluteTolerance(0.05);
		getPIDController().setContinuous(false);
	}
	
    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
    	return inputAngle; // returns the sensor value that is providing the feedback for the system
    }

    protected void usePIDOutput(double output) {
    	if (output < 180)	{
    		motorMultiplier = (((-2.0/45)*output)+1);
    	}
    	else{
    		motorMultiplier = ((-2.0/45)*(-(360-output))+1); // this is where the computed output value fromthe PIDController is applied to the motor

    	}
    }


}
