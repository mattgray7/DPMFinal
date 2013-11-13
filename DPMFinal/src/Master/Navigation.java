package Master;

import java.io.IOException;

import lejos.nxt.*;

public class Navigation extends Thread {

	private NXTRegulatedMotor sensMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	private UltrasonicSensor topUs;
	private UltrasonicSensor bottomUs;
	
	private final double POINT_THRESH = 0.5;
	private final double ANGLE_THRESH = 5.0;
	private final double CORNER_ANGLE_THRESH = .5;
	
	private final int FAST = 200;
	private final int JOG = 150;
	private final int SLOW = 100;
	
	private double LW_RADIUS;
	private double RW_RADIUS;
	private double WHEEL_BASE;
	
	private Odometer odometer;
	public Boolean isTurning = false;
	public Boolean isTraveling = false;
	public Boolean isFinished = false;
	
	private double theta;
	private double gx0 = 60.0;
	private double gx1 = 90.0;
	private double gy0 = 60.0;
	private double gy1 = 90.0;;
	
	private double[] xPath;
	private double[] yPath;

	// test
	private BTSend bts;

	/**
	 * @param odom
	 *            The master odometer shared amongst all classes
	 * @param top
	 *            The top ultrasonic sensor
	 * @param bot
	 *            The bottom ultrasonic sensor
	 */
	public Navigation(Odometer odom, BTSend sender, UltrasonicSensor top,
			UltrasonicSensor bot) {
		topUs = top;
		bottomUs = bot;
		odometer = odom;
		bts = sender;
		LW_RADIUS = odometer.getLeftRadius();
		RW_RADIUS = odometer.getRightRadius();
		WHEEL_BASE = odometer.getWheelBase();
		sensMotor.resetTachoCount();// straight is 0.0, left is -70, right is
									// +70
		this.theta = odometer.getTheta();
	}

	/**
	 * Controls the flow of execution in the Navigation class
	 * 
	 * @return void
	 */
	public void run() {
		// try {bts.sendSignal(2);} catch (IOException e1) {}

		 scan();
		/*travelTo(-15.0, 45.0, false);
		travelTo(45.0, 45.0, false);
		travelTo(45.0, -15.0, false);
		travelTo(0.0, 0.0, false);
		turnTo(90.0, true, true);*/
		// travelTo(100.0, 100.0, false);

		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
		}

	}

	/**
	 * Will travel to the input x and y coordinates while checking for
	 * obstacles. The ignore parameter will not do obstacle detection or
	 * scanning when ignore is set to true. When it is set to false, robot will
	 * constantly be checking for things in front of it.
	 * 
	 * @param x
	 *            The x position to travel to
	 * @param y
	 *            The y position to travel to
	 * @param ignore
	 *            True if the robot should travel to destination without
	 *            obstacle checking
	 * @return void
	 */
	
	 public void travelTo(double x, double y, boolean ignore){ 
		 double minAng;
		 boolean temp = true; 
		 double distance = Math.sqrt(Math.pow(Math.abs(x -odometer.getX()), 2) + Math.pow(Math.abs(y - odometer.getY()), 2));
		 double masterDist = distance; 
		 while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {
	  
			 minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0/ Math.PI); 
			 distance = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()), 2));
	  
	  
			 if (minAng < 0) minAng += 360.0;
	  
			 //Only correct angle during first iteration of the call, otherwise the robot is too oscillatory. 
			 //This is why coordinates are split into 30cmsegments, update every 30cm 
			 if (temp){ 
				 this.turnTo(minAng, true, true); 
				 temp = false;
			 }
	  
			 //after halfway through distance, recorrect angle 
			 if(Math.abs((masterDist / distance) - 2) < 0.5){ 
				 this.turnTo(minAng, true, false);
			 }
	  
	  
			 //this.smoothTurnTo(minAng, x, y); 
			 //this.turnTo(minAng, true);
			 if(Math.abs(minAng - odometer.getTheta()) > 10){ 
				 this.turnTo(minAng,true, false); 
			 }
	  
			 this.setSpeeds(FAST, FAST); 
		 } 
		 this.setSpeeds(0,0); 
	}
	 
	 /**
	* Will turn to the absolute angle passed to it
	* 
	* @param angle
	*            The absolute angle to turn to
	* @param stop
	*            True if the robot should stop after reaching the angle, false
	*            to keep rotating
	* @param corner
	* 			 True if the turn is at a corner, false if correcting angle while traveling
	* @return void
	*/
		
	public void turnTo(double angle, boolean stop, boolean corner) { 
		isTurning = true;
		double error = (angle - this.odometer.getTheta())%360;
		if(corner){
			while (Math.abs(error) > CORNER_ANGLE_THRESH) { 
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
		 }else{
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
				 
		}
		  
		if (stop) { 
			this.setSpeeds(0, 0); 
		} 
		isTurning = false; 
	}
	 



	/**
	 * Will stop and scan 180 degrees with both sensors to check for blocks
	 * 
	 * @return void
	 */
	public void scan() {
		double endAngle = odometer.getTheta() - 90.0;
		turnTo(odometer.getTheta() + 90.0, true, true);
		try {
			Thread.sleep(500);

		} catch (InterruptedException e) {
		}

		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.backward();

		int dist;
		Boolean object = false;
		int objDist = 0;
		// assuming 5 is the max objects the robot can distinguish in one square
		int[][] objects = new int[4][5];
		int objIndex = 0;

		while (odometer.getTheta() > endAngle) {

			dist = getFilteredDistance();

			LCD.drawString("tacho:" + sensMotor.getTachoCount(), 0, 5, false);

			if (dist < 40) {
				if (object == false) {
					// first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;
					objects[1][objIndex] = (int) odometer.getTheta();
					Sound.beep();
				} else {
					// still reading an object
					objDist = dist;
				}
			}

			if ((Math.abs(objDist - dist) > 8) && (object == true)) {
				// end of object
				objects[2][objIndex] = objDist;
				objects[3][objIndex] = (int) odometer.getTheta();
				objIndex++;
				object = false;
				Sound.beep();
			}
		}

		leftMotor.stop();
		rightMotor.stop();

		// if it hasnt read the falling edge of the object, set it
		if (object) {
			objects[2][objIndex] = objDist;
			objects[3][objIndex] = (int) odometer.getTheta();
			Sound.beep();
		}

		int travelDist = 0;
		double travelAng = 0;
		sensMotor.setSpeed(150);
		sensMotor.backward();
		sensMotor.rotate(-70, false);
		sensMotor.stop();

		for (int i = 0; i <= objIndex; i++) {

			travelDist = ((objects[0][i] + objects[2][i]) / 2 - 11); 
			travelAng = ((objects[1][i] + objects[3][i]) / 2.0 + 8.0); 

			if (travelDist > 0) {
				inspect(travelAng, travelDist);
			}
		}

		while (true) {
			LCD.drawString("2ndang:" + travelAng, 0, 6, false);
			LCD.drawString("objs: " + (objIndex + 1), 0, 7, false);
		}

	}

	public void smoothTurnTo(double angle, double x, double y) {
		isTurning = true;
		double error = angle - odometer.getTheta();

		while (Math.abs(error) > ANGLE_THRESH) {
			angle = (Math.atan2(y - odometer.getY(), x - odometer.getX()))
					* (180.0 / Math.PI);
			error = angle - odometer.getTheta();

			if (error < 0) {
				leftMotor.setSpeed(200 + (int) (error * 3.5));
				rightMotor.setSpeed(180);
				Sound.beep();
			} else {
				leftMotor.setSpeed(180);
				rightMotor.setSpeed(200 + (int) (error * 3));
				Sound.beep();
			}
		}
		isTurning = false;
	}
	
	/**
	 * Will generate a path of points that lead from the starting position
	 * to the green zone.
	 * @return void
	 */

	public void generatePath(){
	    double x = odometer.getX();
	    double y = odometer.getY();
	    
	    double x0 = gx0;
	    double y0 = gy0;
	    double x1 = gx1;
	    double y1 = gy1;
	    
	    double xDest = (x0 + x1)/2.0;
	    double yDest = (y0 + y1)/2.0;
	    
	    double[] xpath = new double[20];
	    double[] ypath = new double[20];
	    double length = xpath.length;
	    
	    xpath[0] = x + 30.0;     //first point manually chosen
	    ypath[0] = y;
	    
	    for(int i=0; i< xpath.length-1; i++){
	      if(xpath[i] <= x0){
	        if ((xDest - xpath[i]) > 30.0){
	          xpath[i+1] = xpath[i] + 30.0;
	          ypath[i+1] = ypath[i];
	        }else{
	          xpath[i+1] = xDest;
	          ypath[i+1] = ypath[i];
	          
	          if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	        	  length = i+1;
	        	  break;
	          }
	          
	        }
	      }else{
	        if ((yDest - ypath[i]) > 30.0){
	          ypath[i+1] = ypath[i] + 30.0;
	          xpath[i+1] = xpath[i];
	        }else{
	          ypath[i+1] = yDest;
	          xpath[i+1] = xpath[i];
	          
	          if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	        	  length = i+1;
	        	  break;
	          }
	          
	        }
	      }
	    }
	    
	    for(int i=0; i <= length; i++){
	    	xPath[i] = xpath[i];
	    	yPath[i] = ypath[i];
	    }
	 }
	 

	/**
	 * Blocking method that runs to completion and terminates once the robot is
	 * around the block
	 * 
	 * @return void
	 */
	public void avoid() {

	}

	/**
	 * Constantly checks for obstacle directly in front of robot
	 * 
	 * @return void
	 */
	public void checkForObstacle() {

	}

	/**
	 * Filter for ultrasonic readings, for rotating sensor
	 * 
	 * @return int - The filtered distance reading
	 */
	public int getFilteredDistance() {
		int distance = 0;
		bottomUs.ping();
		if (bottomUs.getDistance() > 200) {
			distance = 200;
		}
		distance = bottomUs.getDistance();
		return distance;
	}

	/**
	 * Once an object is read, this method will take the robot to that block,
	 * then control will be transferred to the slave
	 * 
	 * @return void
	 */
	public void inspect(double angle, int distance) {
		/*
		 * BRING THE CLAW DOWN Use light sensor as distance checker until it
		 * gets close enough. either give control to slave while travelling
		 * forward, then slave tells robot to stop, or we use virtual ports
		 */

		turnTo(angle, true, true);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.rotate(convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();

		// colour check

		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();

	}

	/**
	 * Sets the speeds of the robot
	 * 
	 * @param lSpd
	 *            Speed of left wheel
	 * @param rSpd
	 *            Speed of right wheel
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
	
	/*public void travelTo(double x, double y, boolean ignore) {
	isTraveling = true;
	double heading;
	while (isTraveling) {
		// Takes care of division by zero in tangent function as well as
		// When there is no change in distance on either axis
		if (Math.abs(y - odometer.getY()) <= 2 && (x - odometer.getX()) < 0) {
			heading = 180;
		} else if (Math.abs(y - odometer.getY()) <= 2 && (x - odometer.getX()) > 0) {
			heading = 0;
		} else if (Math.abs(x - odometer.getX()) <= 2 && (y - odometer.getY()) > 0) {
			heading = 90;
		} else if (Math.abs(x - odometer.getX()) <= 2 && (y - odometer.getY()) < 0) {
			heading = 270;
		} else {
			heading = Math.atan((y - odometer.getY()) / (x - odometer.getX())) * (180 / Math.PI);
		}

		// If robot heading is correct, then go straight
		if (Math.abs(heading - theta) <= 10) {
			// Heading is correct then go straight
			leftMotor.setSpeed(FAST);
			rightMotor.setSpeed(FAST);
			leftMotor.forward();
			rightMotor.forward();
		} 
		else if (heading - theta <= 180 && heading - theta >= -180) {
			// Rotate minimum angle
			 turnTo(heading - theta);
		} else if (heading -theta < -180) {
			// Rotate angle + 360
			turnTo(heading - theta+ 360);
		} else {
			// Rotate angle - 360
			turnTo(heading - theta - 360);
		}

		// Update theta
		theta = odometer.getTheta();
		// Break loop if destination is reached
		if (Math.abs(odometer.getX() - x) < 2 && Math.abs(odometer.getY() - y) < 2) {
			Sound.buzz();
			leftMotor.stop();
			rightMotor.stop();
			isTraveling = false;
		}
	}
}*/

/*
 * This method will allow the robot to turn the desired minimum angle All
 * computations are made in the travelTo method this method Just turns the
 * robot
 */
/*public void turnTo(double newTheta) {
	if (newTheta > 1){
	leftMotor.setSpeed(SLOW);
	rightMotor.setSpeed(SLOW);
	leftMotor.rotate(-convertAngle(LW_RADIUS, WHEEL_BASE, newTheta), true);
	rightMotor.rotate(convertAngle(RW_RADIUS, WHEEL_BASE, newTheta), false);
	LCD.drawString("newTheta: " + newTheta, 0, 4, false);
	}
}*/

}
