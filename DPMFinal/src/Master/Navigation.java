package Master;

import lejos.nxt.*;

public class Navigation {

	private NXTRegulatedMotor leftMotor = Motor.A;
	private NXTRegulatedMotor rightMotor = Motor.B;
	
	//constructor
	public Navigation(Odometer odom){
		
	}
	
	//optional method, will control overall flow of navigation. This could be done in the run() method
	//or in the BTSendController class. Both included to give options
	public void controlFlow(){
		
	}
	
	
	/*
	 * Will travel to the input x and y coordinates while checking for obstacles. The
	 * ignore parameter will not do obstacle detection or scanning when ignore is set to
	 * true. When it is set to false, robot will constantly be checking for things in front of it.
	 * Depending on how we want scan() to work (in place or constant), the scan method may
	 * be contained in the navigation class
	 */
	public void travelTo(double x, double y, boolean ignore){
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		leftMotor.forward();
		rightMotor.forward();
		
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
	}
	
	//for 180 degree scan IN PLACE, will modify array of distances??
	public void scan(){
		
	}
	
	//turn to absolute angle
	public void turnTo(double angle){
		
	}
	
	//blocking method that runs to completion and terminates once the robot is around the block
	public void avoid(){
		
	}
	
	//will constantly be checking for obstacle directly in front of robot. is it necessary?
	public void checkForObstacle(){
		
	}
	
	//filter for ultrasonic readings
	public int getFilteredDistance(){
		return 0;
	}
	
	//once an object is read, this method will take the robot to that block, then control
	//will be transferred to the slave. Could be implemented with just travelTo
	public void inspect(){
		
	}
	
	//starts thread. is it necessary? I dont think so
	public void run(){
		
	}
	
	
	
	
	
	
	
}
