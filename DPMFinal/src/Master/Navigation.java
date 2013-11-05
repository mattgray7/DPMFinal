package Master;

import lejos.nxt.*;

public class Navigation {

	private NXTRegulatedMotor leftMotor = Motor.A;
	private NXTRegulatedMotor rightMotor = Motor.B;
	private UltrasonicSensor topUs;
	private UltrasonicSensor bottomUs;
	private final double POINT_THRESH = 1.0;
	private final double ANGLE_THRESH = 1.0;
	private final int FAST = 200;
	private final int SLOW = 100;
	private Odometer odometer;
	
	/**
	 * @param odom	The master odometer shared amongst all classes
	 * @param top	The top ultrasonic sensor
	 * @param bot 	The bottom ultrasonic sensor
	 */
	public Navigation(Odometer odom, UltrasonicSensor top, UltrasonicSensor bot){
		topUs = top;
		bottomUs = bot;
		odometer = odom;
	}
	
	/**
	 * Will travel to the input x and y coordinates while checking for obstacles. The
	 * ignore parameter will not do obstacle detection or scanning when ignore is set to
	 * true. When it is set to false, robot will constantly be checking for things in front of it.
	 * 
	 * @param x 	The x position to travel to
	 * @param y 	The y position to travel to
	 * @param ignore	True if the robot should travel to destination without obstacle checking
	 * @return void
	 */
	public void travelTo(double x, double y, boolean ignore){
		double minAng;
		boolean temp = true;
		while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {
			LCD.drawString("x is:" + Double.toString(x), 0, 5, false);
			LCD.drawString("odx is:" + Double.toString(odometer.getX()), 0, 6, false);
			
			
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);

			if (minAng < 0)
				minAng += 360.0;
			
			//Only correct angle during first iteration of the call, otherwise the robot is too oscillatory.
			//This is why coordinates are split into 30cm segments, update every 30cm
			if (temp){
				this.turnTo(minAng, true);
				temp = false;
			}
			
			this.setSpeeds(FAST, FAST);
		}
		this.setSpeeds(0,0);
	}
	
	/**
	 * Will stop and scan 180 degrees with both sensors to check for blocks
	 * @return void
	 */
	public void scan(){
		
	}
	
	/**
	 * Will turn to the absolute angle passed to it
	 * @param angle	The absolute angle to turn to
	 * @param stop	True if the robot should stop after reaching the angle, false to keep rotating
	 * @return void
	 */
	public void turnTo(double angle, boolean stop) {
		
		
		double error = (angle - this.odometer.getTheta())%360;

		while (Math.abs(error) > ANGLE_THRESH) {		
			error = (angle - this.odometer.getTheta())%360;

			LCD.drawString("minang:" + angle, 0, 4, true);
			
			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}

		if (stop) {
			this.setSpeeds(0, 0);
		}
	}
	
	/**
	 * Blocking method that runs to completion and terminates once the robot is around the block
	 * @return void	
	 */
	public void avoid(){
		
	}
	
	/**
	 * Constantly checks for obstacle directly in front of robot
	 * @return void
	 */
	public void checkForObstacle(){
		
	}
	
	/**
	 * Filter for ultrasonic readings
	 * @return int - The filtered distance reading
	 */
	public int getFilteredDistance(){
		return 0;
	}
	
	/**
	 * Once an object is read, this method will take the robot to that block, then control
	 * will be transferred to the slave
	 * @return void
	 */
	public void inspect(){
		
	}
	
	/**
	 * Controls the flow of execution in the Navigation class
	 * @return void
	 */
	public void run(){
		travelTo(0.0, 30.0, true);
		//turnTo(0.0, true);
		travelTo(30.0, 30.0, true);
		//turnTo(270.0, true);
		travelTo(30.0, 0.0, true);
		//turnTo(180.0, true);
		travelTo(0.0, 0.0, true);
		this.setSpeeds(0,0);
		
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
		
	}
	
	/**
	 * Sets the speeds of the robot
	 * @param lSpd	Speed of left wheel
	 * @param rSpd	Speed of right wheel
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	
	
}
