package Master;

import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.ColorSensor.Color;
import java.util.Random;

public class Navigation extends Thread {

	private NXTRegulatedMotor sensMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;

	private UltrasonicSensor bottomUs;
	private ColorSensor colorSens;
	private ObjectRecognition recog;
	
	private final double POINT_THRESH = 0.5;
	private final double ANGLE_THRESH = 5.0;		//correcting angle thresh
	private final double CORNER_ANGLE_THRESH = .5;		//turning on a point angle thresh
	private final double CLAW_DISTANCE = 17.0;		//the distance the robot must reverse to safely bring down claw
	private final int COLOR_THRESH = 330;			//threshold for colour sensor, >COLOR_THRESH implies object is directly ahead
	
	
	private final int FAST = 200;		//motor speeds
	private final int JOG = 150;
	private final int SLOW = 100;
	
	private int towerHeight = 0;
	private int numTowers = 0;	
	private double LW_RADIUS;
	private double RW_RADIUS;
	private double WHEEL_BASE;
	
	private Odometer odometer;
	public Boolean isBusy = true;			//either inspecting or turning
	//public Boolean isTraveling = false;
	//public Boolean isFinished = false;
	public Boolean hasBlock = false;		//true once a block is read and grabbed
	public Boolean resetPath = false;		//true once a new path needs to be implemented
	public Boolean recentlyAvoided = false;
	
	private double theta;
	private double gx0;			//green zone left x component
	private double gx1;			//green zone right x component
	private double gy0;			//green zone lower y component
	private double gy1;			//green zone upper y component
	
	private double rx0;			//red zone left x component
	private double rx1;			//red zone right x component
	private double ry0;			//red zone lower y component
	private double ry1;			//red zone upper y component
	
	private double wx0 = -30.0;			//left wall
	private double wx1 = 330.0;			//right wall
	private double wy0 = -30.0;			//lower wall
	private double wy1 = 330.0;			//upper wall
	
	private double[] xPath = new double[40];	//the path of x coordinates to travel to
	private double[] yPath = new double[40];	//the path of y coordinate to travel to, should always be synched with xPath
	private int pathIndex = 0;				//used to synchronize x and y path arrays
	
	private double xDestination = 0.0;		//the desired destination at the end of the current path
	private double yDestination = 0.0;	

	int randX;
	int randY;
	double previousX;
	double previousY;

	// test
	private BTSend bts;			//Bluetooth sender class

	/**
	 * Constructor
	 * 
	 * @param odom The shared odometer
	 * @param sender The bluetooth class that sends signals to the master brick
	 * @param bot The bottom ultrasonic sensor
	 * @param cs The color sensor used for object detection and recognition
	 * @param or The object recognition class that computes light readings
	 */
	public Navigation(Odometer odom, BTSend sender, UltrasonicSensor bot, ColorSensor cs, ObjectRecognition or) {
		bottomUs = bot;
		odometer = odom;
		bts = sender;
		colorSens = cs;
		recog = or;
		LW_RADIUS = odometer.getLeftRadius();	//update wheel values
		RW_RADIUS = odometer.getRightRadius();
		WHEEL_BASE = odometer.getWheelBase();
		sensMotor.resetTachoCount();// straight is 0.0, left is -80, right is +80
		this.theta = odometer.getTheta();
	}


	/**
	 * Controls the flow of execution in the Navigation class
	 * 
	 * @return void
	 */
	public void run() {
		rx0 = 30;
		rx1 = 60;
		ry0 = 30;
		ry1 = 60;
		randomPathFinder();
		
		//hardcoded for demo, move away from surrounding walls so they aren't picked up during scan
		//travelTo(30.0, 30.0);
		//turnTo(90.0, true, true);
		//travelTo(30.0, 30.0);
		//turnTo(90.0, true, true);
		
		//travelTo(0, 60.0);
		/*travelTo(60, 60);
		travelTo(60, 0);
		travelTo(0,0);
		turnTo(90, true, false);*/
		
		//generate linear path to the green zone
		//generatePath();
		
		//180 degree scan for objects
		//scan();
		
		//for loop starts at 1 since first elements are current position
		/*for(int i=1; i < xPath.length; i++){
			//if a block is not being transported, continue searching
			if(!hasBlock){
				travelTo(xPath[i], yPath[i]);
				pathIndex = i;

				//if the robot is close enough to it's destination, break our of loop
				if((Math.abs(odometer.getX() - xDestination) < POINT_THRESH + 10) 
						&& (Math.abs(odometer.getY() - yDestination) < POINT_THRESH + 10)){
					Sound.buzz();
					break;
				}
				
				//xpath and ypath have already been updated, restart for loop from beginning for new path
				if(resetPath){
					i=1;
					resetPath = false;
				}
				
				//do a scan in between traveling poitns
				scan();
			}else{
				//block is held
				break;	//only for demo, will need to be handled for final demo
			}
		}*/
		/*
		while(true){
		int pathChoice = checkCurrentPath();
		if(pathChoice == 1){
			//continue straight, travel 30cm. If no obstacle in way, scan, otherwise, 
			//the robot will have turned 90 degrees and next iteration of while loop should check current
			//position with new heading
			if(Math.abs(odometer.getTheta()) < ANGLE_THRESH)){
				travelTo(odometer.getX()+30.0, odometer.getY());
				if(!recentlyAvoided){
					scan();
				}else{
					recentlyAvoided = false;
					continue;	
				}
			}else if(Math.abs(odometer.getTheta() - 90) < ANGLE_THRESH){
				travelTo(odometer.getX(), odometer.getY() + 30.0);
				if(!recentlyAvoided){
					scan();
				}else{
					recentlyAvoided = false;
					continue;	
				}
			}else if(Math.abs(odometer.getTheta() - 180) < ANGLE_THRESH){
				travelTo(odometer.getX() - 30.0, odometer.getY());
				if(!recentlyAvoided){
					scan();
				}else{
					recentlyAvoided = false;
					continue;	
				}
			}else{
				travelTo(odometer.getX(), odometer.getY() - 30.0);
				if(!recentlyAvoided){
					scan();
				}else{
					recentlyAvoided = false;
					continue;	
				}
			}
		}else if(pathChoice == 2){
			if(Math.abs(odometer.getTheta()) < ANGLE_THRESH){
				travelTo(wx1-45.0, odometer.getY());	//will update recently avoided
				if(!recentlyAvoided){
					if(!scan()){
						//nothing was found, calculate new path and continue
						calculateNewPath();//turn left or right, stops once turned
					}else{
						//iteration ends, next iteration will check position facing new direction
						continue;
					}

				}else{
					recentlyAvoided = false;
					continue;	
				}
			}else if(Math.abs(odometer.getTheta() - 90) < ANGLE_THRESH){
				travelTo(odometer.getX(), wy1 - 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else if(Math.abs(odometer.getTheta() - 180) < ANGLE_THRESH){
				travelTo(wx0 + 45.0, odometer.getY());
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else{
				travelTo(odometer.getX(), wy0 + 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}
		}else if(pathChoice == 3){
			if(Math.abs(odometer.getTheta()) < ANGLE_THRESH){
				travelTo(rx0 - 45.0, odometer.getY());	//will update recently avoided
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else if(Math.abs(odometer.getTheta() - 90) < ANGLE_THRESH){
				travelTo(odometer.getX(), ry0 - 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else if(Math.abs(odometer.getTheta() - 180) < ANGLE_THRESH){
				travelTo(rx1 + 45.0, odometer.getY());
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else{
				travelTo(odometer.getX(), ry1 + 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}
		}else if (pathChoice == 4){
			if(Math.abs(odometer.getTheta()) < ANGLE_THRESH){
				travelTo(gx0 - 45.0, odometer.getY());
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else if(Math.abs(odometer.getTheta() - 90) < ANGLE_THRESH){
				travelTo(odometer.getX(), gy0 - 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else if(Math.abs(odometer.getTheta() - 180) < ANGLE_THRESH){
				travelTo(gx1 + 45.0, odometer.getY());
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}else{
				travelTo(odometer.getX(), gy1 + 45.0);
				if(!recentlyAvoided){
					if(!scan()){
						calculateNewPath();
					}else{
						continue;
				}else{
					recentlyAvoided = false;
					continue;
				}
			}
		}
		}//end of while
		*/
		
	}
	

	/**
	 * Will travel to the input x and y coordinates while checking for
	 * obstacles. Correct's it's orientation if the angle difference is large enough.
	 * 
	 * @param x
	 *            The x position to travel to
	 * @param y
	 *            The y position to travel to
	 * @return void
	 */
	 public void travelTo(double x, double y){ 
		
		 isBusy = false;		 //odometer should be corrected while traveling, only time where robot is not busy is when travelling
		 boolean first = true; 		//true only for first iteration of while loop, turn to minAng with greater accuracy
		 double minAng;
		 double masterAng = 0;		//the desired angle the robot should be traveling at
		 colorSens.setFloodlight(Color.RED);		//turn on floodlight for obstacle detection
		 double distance = Math.sqrt(Math.pow(Math.abs(x -odometer.getX()), 2) + Math.pow(Math.abs(y - odometer.getY()), 2));	//pythagorean theorum	
		 double masterDist = distance;		//distance the robot will have to travel
		 
		 while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {
	  
			 minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0/ Math.PI); 	//update minimum angle
			 distance = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()), 2));	//update distance traveled
	  
			 //object detected immediately in front of robot
			 if(colorSens.getNormalizedLightValue() > COLOR_THRESH - 10){
				 this.setSpeeds(0,0);
				 recentlyAvoided = true;
				 //begin inspection/avoidance, robot is busy
				 isBusy = true;
				 
				 //check the color
				 if(recog.checkColor()){
					//capture the block
					 capture();		
				 }else{
					 //avoid is recursive, input tells which iteration of recursion
					// if(hasBlock){
						 avoid();
					 //}else{
						 //calculateNewPath();
						 //return;
					 //}
				 }
				 //both capture and avoid will move the robot past it's next destination, break this travelTo call
				 return;
			 }
			 
			 if (minAng < 0) minAng += 360.0;
	  
			 //Only turn with a small threshold for the first iteration, otherwise the robot is too oscillatory
			 if (first){ 
				 this.turnTo(minAng, true, true);
				 masterAng = minAng;
				 first = false;
			 }
	  
			 //after halfway through distance, correct angle 
			 if(Math.abs((masterDist / distance) - 2) < POINT_THRESH){ 
				 this.turnTo(minAng, true, false);
			 }
	  
			 //if angle difference is too large, need to correct
			 if(Math.abs(minAng - odometer.getTheta()) > 10){ 
				 this.turnTo(minAng,true, false); 
			 }
	  
			 this.setSpeeds(FAST, FAST);
			 //this.setSpeeds(SLOW,  SLOW); // For testing
		 } 
		 this.setSpeeds(0,0); 
		 turnTo(masterAng, true, true);
		 isBusy = true;
		 colorSens.setFloodlight(false);
	}
	 
	 
	/**
	 * Turn to the absolute angle given with 2 levels of accuracy	
	 * @param angle The absolute angle to turn to
	 * @param stop True if the robot should stop spinning after turning
	 * @param corner True if the robot should turn with higher accuracy, used for turning on point to new location
	 */
	public void turnTo(double angle, boolean stop, boolean corner) { 
		//don't correct odometer during turns
		isBusy = true;
		double error = (angle - this.odometer.getTheta()) % 360;
		
		//turn with higher accuracy --> narrower angle threshold
		if(corner){
			while (Math.abs(error) > CORNER_ANGLE_THRESH) { 
				error = (angle - this.odometer.getTheta()) % 360;
				
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
			//for correcting angle while traveling
			while (Math.abs(error) > ANGLE_THRESH) { 
				error = (angle - this.odometer.getTheta()) % 360;
				
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
		
		//stop the spinnign
		if (stop) { 
			this.setSpeeds(0, 0); 
		} 
		
		//no longer turning, so now longer busy
		isBusy = false; 
	}


	/**
	 * Will stop and scan 180 degrees in an attempt to detect objects
	 * 
	 * @return true If a blue block was found
	 * @return false If no blue block was found
	 */
	public Boolean scan() {
		double endAngle = odometer.getTheta() - 90.0;	//set end angle of scan
		turnTo(odometer.getTheta() + 90.0, true, true);	//turn to start angle of scan
		
		try {Thread.sleep(500);} catch (InterruptedException e) {}		//wait for half a second so ultrasonic sesnor can stabilize
		
		//start rotating
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.backward();

		int dist;							//distance read from us sensor
		Boolean object = false;				//true when scan is passing over an object, false in between
		int objDist = 0;					//previously read distance of object
		
		int[][] objects = new int[4][20];	//assuming 20 is the max objects the robot can distinguish in one square
		int objIndex = 0;					//synchronizes the object distance and angle in the 2D array

		while (odometer.getTheta() > endAngle) {
			dist = getFilteredDistance();		//update the distance

			//if the reading is within a 32cm radius
			if (dist < 32) {
				if (object == false) {
					// first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;		//store the distance of the first edge
					objects[1][objIndex] = (int) odometer.getTheta();		//store the angle of the first edge
					Sound.beep();
				} else {
					// still reading the same object, update the object distance
					objDist = dist;
				}
			}
			
			//if there is a difference >9cm between the previously read distance and the new distance, it is the end of an object
			if ((Math.abs(objDist - dist) > 9) && (object == true)) {
				//end of object, all data known for that object
				objects[2][objIndex] = objDist;
				objects[3][objIndex] = (int) odometer.getTheta();
				objIndex++;
				object = false;
				Sound.beep();
			}
		}

		leftMotor.stop();
		rightMotor.stop();

		// if scan ends without reading the falling edge of the object, set it
		if (object) {
			objects[2][objIndex] = objDist;
			objects[3][objIndex] = (int) odometer.getTheta();
			Sound.beep();
		}

		int travelDist = 0;
		double travelAng = 0;
		double ANGLE_OFFSET;
		
		//for each object read
		for (int i = 0; i <= objIndex; i++) {
			//distance is the average of the distances read at each edge
			double avgDist = (objects[0][i] + objects[2][i]) / 2.0;
			travelDist = (int)avgDist- 10; //-10 for distance between sensor and wheel base
			
			//Depending on distance read, angle offset needs to be added to the new angle
			if(avgDist < 5){
				ANGLE_OFFSET = 4;
			}else if (avgDist <= 15){
				ANGLE_OFFSET = 3;
			}else if (avgDist <= 20){
				ANGLE_OFFSET = 1;
			}else if (avgDist <= 25){
				ANGLE_OFFSET = 0;
			}else{
				ANGLE_OFFSET = -0.5;
			}
			
			//angle to travel at
			travelAng = ((objects[1][i] + objects[3][i]) / 2.0 + ANGLE_OFFSET); 
			
			//checking for bad distance readings
			if (travelDist > 0) {

				colorSens.setFloodlight(Color.RED);
				boolean found = inspect(travelAng, travelDist);
				if(found){
					return true;
				}


			}
		}
		return false;
	}
	
	/**
	 * Will drive the given distance at the given angle and check the color of an object
	 * @param angle The angle to drive at
	 * @param distance The distance the object was sensed at
	 */
	public Boolean inspect(double angle, int distance) {
		double x = odometer.getX();
		double y = odometer.getY();
		double distTraveled = 0;
		turnTo(angle, true, true);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.forward();
		
		while(colorSens.getNormalizedLightValue() < COLOR_THRESH){
			//update the distance traveled
			distTraveled = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()),2));
			
			//if you have reached the distance, stop and inspect
			if(Math.abs(distance - distTraveled) < 0.5){
				break;
			}
			
		}
		
		distTraveled = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()),2));
		
		leftMotor.stop();
		rightMotor.stop();
		//try {Thread.sleep(100);} catch (InterruptedException e) {}
		
		//check the color
		if(recog.checkColor()){
			Sound.beep();
			hasBlock = true;
			capture();	
			return true;
		}else{
			rotateSensorsRight(15);
			
			if(recog.checkColor()){
				//if after the second reading, a blue block was sensed, return the sensor to it's initial position
				rotateSensorsLeft(15);
				
				//and capture the block
				Sound.beep();
				hasBlock = true;
				capture();	
				return true;
			}else{
				//first two sensor checks read wood blocks, rotate light sensor 30 degrees to the 
				//left (15 degrees from starting orientation)
				rotateSensorsLeft(30);
				
				if(recog.checkColor()){
					//rotate sensor back to initial position
					rotateSensorsRight(15);
					
					//and capture the block
					Sound.beep();
					hasBlock = true;
					capture();
					return true;
				}else{
					//all checks read object as wood block, return sensor to starting position
					rotateSensorsRight(15);
					
					//reverse robot to starting position of inspection
					leftMotor.rotate(-convertDistance(LW_RADIUS, distTraveled), true);
					rightMotor.rotate(-convertDistance(RW_RADIUS, distTraveled), false);
					leftMotor.stop();
					rightMotor.stop();
					return false;
				}
			}
		}

	}
	
	/**
	 * Assumes a blue block is in front of the robot, will reverse, lower claw, and grab and lift the block
	 */
	public void capture(){
		double distance = 18.0;
		
		//reverse far enough to lower claw
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();
		
		//move sensors out of the way of the claw
		rotateSensorsRight(80);
		
		//send signal from BTSend class to lower arms and open the claw (this signal is 1)
		try {bts.sendSignal(1);} catch (IOException e) {}	
		
		//wait for claw to come down
		try {Thread.sleep(2500);} catch (InterruptedException e1) {}
		
		//travel back to the block and slightly farther to position block in claw
		leftMotor.rotate(convertDistance(LW_RADIUS, distance+4), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, distance+4), false);
		
		/*//Lifting sometimes fails for blocks at an angle, following code "wiggles" the claw with the blue
		//block in it in an attempt to straighten out the block for lifting.
		//rotate right
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.backward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		
		//rotate left
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.backward();
		rightMotor.forward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		
		//go forward 5 more cm to ensure block is in claw and oriented properly
		//MAY NEED TO BE CHANGED/CHECKED with second ultrasonic to see if there's anything behind the block
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.rotate(convertDistance(LW_RADIUS, 5), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, 5), false);*/
		
		leftMotor.setSpeed(JOG+100);
		rightMotor.setSpeed(JOG);
		leftMotor.forward();
		rightMotor.forward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		
		//rotate left
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG+100);
		leftMotor.forward();
		rightMotor.forward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		
		//go forward 5 more cm to ensure block is in claw and oriented properly
		//MAY NEED TO BE CHANGED/CHECKED with second ultrasonic to see if there's anything behind the block
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.rotate(convertDistance(LW_RADIUS, 5), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, 5), false);
		
		leftMotor.stop();
		rightMotor.stop();
		
		//Send signal to clamp and lift the object
		try {bts.sendSignal(2);} catch (IOException e) {Sound.buzz();}	//2 for clamp and raise arms
		//try {bts.sendSignal(3);} catch (IOException e) {Sound.buzz();}	//3 for clamping only
		
		//wait for lift and clamp
		try {Thread.sleep(1700);} catch (InterruptedException e1) {}
		
		//reverse far enough to lower claw
		/*leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);*/
		
		//try {bts.sendSignal(4);} catch (IOException e) {Sound.buzz();}	//4 for raising arms
		
		//try {Thread.sleep(1700);} catch (InterruptedException e1) {}
		
		//return sensors to original orientation
		rotateSensorsLeft(80);
		
		//drive and deposit in green zone.
		finishLine();
	}
	
	/**
	 * Will take the robot from it's current position directly to the bottom left corner of the green zone
	 */
	public void finishLine(){
		hasBlock = false;
		double x = odometer.getX();
		double y = odometer.getY();
		
		//depending on location in relation to green zone, travel to a green zone corner and turn to the center
		/*if((x <= gx0) && (y <=gy0)){

				travelTo(gx0, gy0);
				turnTo(45, true, true);
		}else if ((x <= gx0) && (y >= gy1)){
				travelTo(gx0, gy1);
				turnTo(-45, true, true);
			
		}else if ((x >= gx1) && (y <= gy0)){
			travelTo(gx1, gy0);
			turnTo(135, true, true);
		}else if ((x >= gx1) && (y >= gy1)){
			travelTo(gx1, gy1);
			turnTo(225, true, true);
		}else{
			//default to bottom left corner
			travelTo(gx0, gy0);
			turnTo(45, true, true);
		}*/
		
		if(towerHeight == 0){
			travelTo(gx0, gy0);
			turnTo(45.0, true, true);
		}else if(towerHeight == 1){
			travelTo(gx0 - 5.6, gy0 - 5.6);
			turnTo(45.0, true, true);
		}else if (towerHeight == 2){
			travelTo(gx0 - 7, gy0 - 7);
			turnTo(25.0, true, true);
		}
		
		//rotate sensors away from claw
		rotateSensorsRight(80);
		
		//send signal to lower arms all the way and open claw - MAY NEED TO BE CHANGED FOR TOWER BUILDING
		if(towerHeight == 0){
			try {bts.sendSignal(1);} catch (IOException e) {}
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}
			
			leftMotor.setSpeed(SLOW);
			rightMotor.setSpeed(SLOW);
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.rotate(-convertDistance(LW_RADIUS, 12), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, 12), false);
			
			
			try {bts.sendSignal(2);} catch (IOException e) {}	//raise arms to max
			try {Thread.sleep(2000);} catch (InterruptedException e1) {}
			
		}else if (towerHeight == 1){
			try {bts.sendSignal(10);} catch (IOException e) {}	
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}
			
			//reverse far enough to raise claw to the top again
			leftMotor.setSpeed(SLOW);
			rightMotor.setSpeed(SLOW);
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.rotate(-convertDistance(LW_RADIUS, 10), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, 10), false);
			
			try {bts.sendSignal(-10);} catch (IOException e) {}
			try {Thread.sleep(1000);} catch (InterruptedException e1) {}
		}else if (towerHeight == 2){
			try {bts.sendSignal(11);} catch (IOException e) {}	
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}
			
			//reverse far enough to raise claw to the top again
			leftMotor.setSpeed(SLOW);
			rightMotor.setSpeed(SLOW);
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.rotate(-convertDistance(LW_RADIUS, 10), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, 10), false);
			
			try {bts.sendSignal(-11);} catch (IOException e) {}
			try {Thread.sleep(1000);} catch (InterruptedException e1) {}
			
		}
		towerHeight++;
		if(towerHeight == 3){
			towerHeight = 0;
			numTowers++;
		}
		rotateSensorsLeft(80);
	}


	
	/**
	 * Will generate a path of points that lead from current location
	 * to the green zone.
	 * @return void
	 */

	public void generatePath(){
		//method will need to be changed to handle walls and green zone, not going to comment until algorithm is complete
	    double x = odometer.getX();
	    double y = odometer.getY();
	    
	    double x0 = gx0;
	    double y0 = gy0;
	    double x1 = gx1;
	    double y1 = gy1;
	    
	    double xDest = (x0 + x1)/2.0;
	    double yDest = (y0 + y1)/2.0;
	    
	    double[] xpath = new double[40];
	    double[] ypath = new double[40];
	    double length = xpath.length;
	    
	    //should include x+3-, y+30, but currently does not
	    
	    xpath[0] = x;     //first point manually chosen
	    ypath[0] = y;

	    
	    for(int i=0; i< xpath.length - 1; i++){
	        
	        //left and below green zone
	        
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
	        }else if (xpath[i] >= x1){
	          if ((xpath[i] - xDest) > 30.0){
	            xpath[i+1] = xpath[i] - 30.0;
	            ypath[i+1] = ypath[i];
	          }else{
	            xpath[i+1] = xDest;
	            ypath[i+1] = ypath[i];
	            
	            if(ypath[i+1] == yDest && xpath[i+1] == xDest){
	            	  length = i+1;
	            	  break;
	            }
	            
	          }
	        }else if (ypath[i] <= y0){
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
	        }else{
	          if ((ypath[i] - yDest) >= 30.0){
	            ypath[i+1] = ypath[i] - 30.0;
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
	    

	    xDestination = xDest;
	    yDestination = yDest;
	    
	    for(int i=0; i < length; i++){
	    	xPath[i] = xpath[i];
	    	yPath[i] = ypath[i];
	    }
	 }
	 

	/**
	 * Recursive obstacle avoidance that returns once the robot is successfully around the obstacle
	 * @param i The iteration of recursion
	 */
	public void avoid() {
		//values for p-type wall following
		int BAND_CENTER = 20;
		int error = 0;
		
		//dont correct during avoidance
		isBusy = true;
		
		//reverse away from object
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.rotate(-convertDistance(LW_RADIUS, 8.0), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 8.0), false);
		leftMotor.stop();
		rightMotor.stop();
		
		//angle before avoidance
		double initAngle = odometer.getTheta();
		
		//turn 90 degrees left
		turnTo(odometer.getTheta() + 90.0, true, true);
		
		//ping us sensor
		int dist = getFilteredDistance();

		if (dist > 30){
			//no object within 30cm to the left, avoid this direction
			rotateSensorsRight(40);
			wallFollowLeft(initAngle);
			rotateSensorsLeft(40);

		}else{
			//turn to the right
			turnTo(odometer.getTheta() - 180.0, true, true);
			dist = getFilteredDistance();
			
			//check the distance to the right
			if(dist > 30){
				//nothing within 30cm, avoid to the right
				rotateSensorsLeft(40);
				wallFollowRight(initAngle);
				rotateSensorsRight(40);

			}else{
				//turn to initial angle and reverse 30cm backward
				turnTo(initAngle-180, true, true);
				rotateSensorsRight(40);
				leftMotor.setSpeed(FAST);
				rightMotor.setSpeed(FAST);
				leftMotor.forward();
				rightMotor.forward();

				
				wallFollowLeft(initAngle);
				rotateSensorsLeft(40);
				try {Thread.sleep(1000);} catch (InterruptedException e) {}
			}
		}

		//if not carrying a block, regenerate path to green zone
		if(!hasBlock){
			generatePath();		
			resetPath = true;	
		}else{
			//has a block, drive to the green zone
			finishLine();
		}

	}
	
	public void wallFollowLeft(double initAngle){

		//start driving
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();
		

		int BAND_CENTER = 20;
		int BAND_WIDTH = 2;
		int dist;
		int error;
		while(true){
			dist = getFilteredDistance();
			error = BAND_CENTER - dist;
			
			//cap the error so the speed doesn't go crazy
			if(error > 100){
				error = 50;
			}else if (error < -100){
				error = -50;
			}
			
			if(error < -BAND_WIDTH){
				//too far away, need to turn right
				leftMotor.setSpeed(FAST + 75);
				rightMotor.setSpeed(FAST + 50 + (error*2));	//error is negative, right move slower
			}else if (error > BAND_WIDTH){
				//too close, need to turn left
				leftMotor.setSpeed(FAST + 75);
				rightMotor.setSpeed(FAST + 50 + (error * 2));	//error is positive, right moves faster
			}else{
				leftMotor.setSpeed(FAST + 100);
				rightMotor.setSpeed(FAST + 100);
			}
			
			//if the robot's orientation reaches its initial angle - 20 degrees, wall is assumed to be passed
			if (Math.abs(odometer.getTheta() - (initAngle - 20)) < ANGLE_THRESH){
				Sound.buzz();
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				break;
			}
		}
		
		
		//drive straight for 1.5s to get sufficiently apst the block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}
	}
	
	public void wallFollowRight(double initAngle){
		
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();
		
		int BAND_CENTER = 20;
		int BAND_WIDTH = 2;
		int dist;
		int error;
		//actual wall following
		while(true){
			dist = getFilteredDistance();
			error = BAND_CENTER - dist;
			
			if(error > 100){
				error = 50;
			}else if (error < -100){
				error = -50;
			}
			//if robot gets to starting x orientation, it has successfully avoided the block
			
			if(error < -BAND_WIDTH){
				//too far away, need to turn left
				leftMotor.setSpeed(FAST + 50 + (error*2));	//error is negative, left moves slower
				rightMotor.setSpeed(FAST + 75);	
			}else if (error > BAND_WIDTH){
				//too close, need to turn right
				leftMotor.setSpeed(FAST + 50 + (error*2));	//error is positive, left moves faster
				rightMotor.setSpeed(FAST + 75);
			}else{
				//within band, go straight
				leftMotor.setSpeed(FAST + 50);
				rightMotor.setSpeed(FAST + 50);
			}
			
			//if it reaches int's initial angle + 20 degrees, assumed avoided object
			if (Math.abs(odometer.getTheta() - (initAngle + 20)) < ANGLE_THRESH){
				Sound.buzz();
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				break;
			}
			
		}

		//drive forward to get past block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}
		
		
	}


	/**
	 * Filter for ultrasonic readings, for rotating sensor
	 * 
	 * @return The filtered distance reading
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
	
	public int determineAvoidDirection(){
		
		if(Math.abs(odometer.getTheta()) < ANGLE_THRESH){
			if(odometer.getY() < ry1 && odometer.getY() > ry0){
				//robot is in between y values of red zone
				if(odometer.getX() > rx1){
					//on the right side of the red zone
				}
			}
		}
		
		return 0;
	}
	
	/**
	 * Rotates sensor motor clockwise 80 degrees
	 */
	public void rotateSensorsRight(int angle){
		sensMotor.setSpeed(150);
		sensMotor.forward();
		sensMotor.rotate(angle, false);
		sensMotor.stop();
	}
	
	/**
	 * Rotates sensor motor counter-clockwise 80 degrees
	 */
	public void rotateSensorsLeft(int angle){
		sensMotor.setSpeed(150);
		sensMotor.backward();
		sensMotor.rotate(-angle, false);
		sensMotor.stop();
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
	
	/**
	 * Converts a distance to radians
	 * @param radius Radius of wheel
	 * @param distance Distance to be converted
	 * @return The radian value of the input distance
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	
	/**
	 * Converts the angle that the robot should turn to into specific rotation angle for a single motor
	 * @param radius The radius of the wheel
	 * @param width The distance between the two wheels
	 * @param angle The angle to be converted
	 * @return The degrees that the wheel should rotate so that the entire robot rotates the input angle
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	/**
	 * Set the bottom left coordinate of the green zone
	 * @param x The x coordinate of the corner
	 */
	public void setGX0(int x){
		this.gx0 = x;
	}
	
	/**
	 * Set the top left coordinate of the green zone
	 * @param y The y coordinate of the corner
	 */
	public void setGY0(int y){
		this.gy0 = y;
	}
	
	/**
	 * Set the bottom right coordinate of the green zone
	 * @param x The x coordinate of the corner
	 */
	public void setGX1(int x){
		this.gx1 = x;
	}
	
	/**
	 * Set the upper right coordinate of the green zone
	 * @param y The y coordinate of the corner
	 */
	public void setGY1(int y){
		this.gy1 = y;
	}
	
	public boolean isBusy() {
		return isBusy;
	}
	
	public void setBusy(boolean busy) {
		isBusy = busy;
	}
	public void randomPathFinder() {
		boolean goodPath;
		Random x = new Random();
		for (int i = 0; i< 100000000; i++) {
			randX = x.nextInt(75);
			randY = x.nextInt(75);
			//If destination coordinate is in red zone, generate another point
			if (randX >= rx0 && randX <= rx1 && randY >= ry0 && randY <= ry1) {
				randX = x.nextInt(75);
				randY = x.nextInt(75);
			}
			goodPath = checkPath(randX, randY);
			if (goodPath) {
				travelTo(randX, randY);
				//scan();
			}
		} 
	}
	public boolean checkPath(int x, int y) {
		double currentX = odometer.getX();
		double currentY = odometer.getY();
		double heading = Math.atan2(y - odometer.getY(), x - odometer.getX()) * (180.0/ Math.PI);
		double testY;
		//Check if any point along path intersects the red zone
		if (x - currentX > 0) {
			while (currentX <= x) {
				testY = currentX * Math.tan(heading * Math.PI/180);
				if (currentX >= rx0-25 && currentX <= rx1 + 25 && testY >= ry0 - 25 && testY <= ry1 + 25) {
					return false;
				}
				else {
					currentX++;
				}
			}
		}
		else {
			while (x <= currentX) {
				testY = currentX * Math.tan((180 - heading) * Math.PI/180);
				if (currentX >= rx0-25 && currentX <= rx1 + 25 && testY >= ry0 - 25 && testY <= ry1 + 25) {
					return false;
				}
				else {
					currentX--;
				}
			}
		}
		return true;
	}
}
