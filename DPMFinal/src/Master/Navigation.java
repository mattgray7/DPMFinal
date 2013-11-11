package Master;

import java.io.IOException;

import lejos.nxt.*;

public class Navigation extends Thread{
	
	private NXTRegulatedMotor sensMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	private UltrasonicSensor topUs;
	private UltrasonicSensor bottomUs;
	private final double POINT_THRESH = 1.0;
	private final double ANGLE_THRESH = 0.5;
	private final int FAST = 200;
	private final int JOG = 150;
	private final int SLOW = 100;
	private double LW_RADIUS;
	private double RW_RADIUS;
	private double WHEEL_BASE;
	private Odometer odometer;
	public Boolean isTurning = false;
	
	private BTSend bts;
	
	/**
	 * @param odom	The master odometer shared amongst all classes
	 * @param top	The top ultrasonic sensor
	 * @param bot 	The bottom ultrasonic sensor
	 */
	public Navigation(Odometer odom, BTSend sender, UltrasonicSensor top, UltrasonicSensor bot){
		topUs = top;
		bottomUs = bot;
		odometer = odom;
		bts = sender;
		LW_RADIUS = odometer.LW_RADIUS;
		RW_RADIUS = odometer.RW_RADIUS;
		WHEEL_BASE = odometer.WHEEL_BASE;
		sensMotor.resetTachoCount();//straight is 0.0, left is -70, right is +70
	}
	
	
	/**
	 * Controls the flow of execution in the Navigation class
	 * @return void
	 */
	public void run(){
		try {bts.sendSignal(2);} catch (IOException e1) {}
		//scan();
		/*travelTo(0.0, 60.0, false);
		travelTo(60.0, 60.0, false);
		travelTo(60.0, 0.0, false);
		travelTo(0.0, 0.0, false);*/
		//travelTo(100.0, 100.0, false);
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
		
		//turnTo(75.0, true);
		/*turnTo(50.0, true);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
		turnTo(13.0, true);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
		turnTo(0.0, true);
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();*/
		
		try { Thread.sleep(5000); }catch (InterruptedException e) {}
		
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
			
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);

			if (minAng < 0)
				minAng += 360.0;
			
			//Only correct angle during first iteration of the call, otherwise the robot is too oscillatory.
			//This is why coordinates are split into 30cm segments, update every 30cm
			if (temp){
				this.turnTo(minAng, true);
				temp = false;
			}
			
			
			//this.smoothTurnTo(minAng, x, y);
			this.turnTo(minAng, true);
			/*if(Math.abs(minAng - odometer.getTheta()) > 5){
				this.turnTo(minAng, true);
			}*/
				//temp = false;
			//}
			
			this.setSpeeds(FAST, FAST);
		}
		this.setSpeeds(0,0);
	}
	
	/**
	 * Will stop and scan 180 degrees with both sensors to check for blocks
	 * @return void
	 */
	public void scan(){
		//rotate to the left
		Boolean object = false;
		int objDist = 0;
		
		//assuming 5 is the max objects the robot can distinguish in one square
		int[][] objects = new int[4][5];
		int objIndex = 0;
		
		sensMotor.setSpeed(150);
		sensMotor.backward();
		sensMotor.rotate(-70, false);
		sensMotor.stop();
		
		sensMotor.setSpeed(20);
		sensMotor.forward();
		sensMotor.rotate(140, true);
		
		int dist;
		
		while(sensMotor.getTachoCount() < 70){
			dist = getFilteredDistance();
			
			LCD.drawString("tacho:" + sensMotor.getTachoCount(), 0, 5, false);
			
			if(dist < 40){
				if (object == false){
					//first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;
					objects[1][objIndex] = sensMotor.getTachoCount();
					Sound.beep();
				}else{
					//still reading an object
					objDist = dist;
				}
			}
			
			if((Math.abs(objDist - dist) > 4) && (object == true)){
				//end of object
				objects[2][objIndex] = objDist;
				objects[3][objIndex] = sensMotor.getTachoCount();
				objIndex++;
				object = false;
				Sound.beep();
			}
		}
		
		//if it hasnt read the falling edge of the object, set it
		if(object){
			objects[2][objIndex] = objDist;
			objects[3][objIndex] = 90;
			objIndex++;
		}
		
		int travelDist = 0;
		double travelAng = 0;
		
		for(int i=0; i <= objIndex; i++){
			
			travelDist = ((objects[0][i] + objects[2][i])/2 - 11);	// average - the distance from claw to block
			travelAng = ((objects[1][i] + objects[3][i])/2.0);	//average of two angles, 5 for tweaking
			
			if(objIndex > 0){
				inspect((odometer.getTheta() - travelAng), travelDist);
			}
			
			
		}
		
		while(true){
			LCD.drawString("2ndang:" + (odometer.getTheta() - travelAng), 0, 6, false);
			LCD.drawString("objs: " + (objIndex+1), 0, 7, false);
		}

		
	}
	
	public void smoothTurnTo(double angle, double x, double y){
		isTurning = true;
		double error = angle - odometer.getTheta();

		while (Math.abs(error) > ANGLE_THRESH) {		
			angle = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
			error = angle - odometer.getTheta();
			
			if (error < 0){
				leftMotor.setSpeed(200 + (int)(error * 3.5));
				rightMotor.setSpeed(180);
				Sound.beep();
			}else{
				leftMotor.setSpeed(180);
				rightMotor.setSpeed(200 + (int)(error * 3));
				Sound.beep();
			}
		}
		isTurning = false;
	}
	
	/**
	 * Will turn to the absolute angle passed to it
	 * @param angle	The absolute angle to turn to
	 * @param stop	True if the robot should stop after reaching the angle, false to keep rotating
	 * @return void
	 */
	public void turnTo(double angle, boolean stop) {
		isTurning = true;
		
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
		isTurning = false;
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
	 * Filter for ultrasonic readings, for rotating sensor
	 * @return int - The filtered distance reading
	 */
	public int getFilteredDistance(){
		int distance=0;
		bottomUs.ping();
		if (bottomUs.getDistance() > 200){
			distance = 200;
		}
		distance = bottomUs.getDistance();
		return distance;
	}
	
	/**
	 * Once an object is read, this method will take the robot to that block, then control
	 * will be transferred to the slave
	 * @return void
	 */
	public void inspect(double angle, int distance){
		/*BRING THE CLAW DOWN
		Use light sensor as distance checker until it gets close enough.
		either give control to slave while travelling forward, then slave tells robot
		to stop, or we use virtual ports
		 */
		
		
		
		turnTo(angle, true);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.rotate(convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();
		
		//colour check
		
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();
		
		
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
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
}
