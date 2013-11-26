package Master;

import java.io.IOException;
import java.util.Random;

import lejos.nxt.ColorSensor;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class Navigation extends Thread {
	private NXTRegulatedMotor sensMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;

	private final int FAST = 200;
	private final int JOG = 150;
	private final int SLOW = 100;

	private final double POINT_THRESH = 0.5;
	private final double ANGLE_THRESH = 5.0;                //correcting angle thresh
	private final double CORNER_ANGLE_THRESH = .5;                //turning on a point angle thresh
	private final double CLAW_DISTANCE = 17.0;                //the distance the robot must reverse to safely bring down claw
	private final int COLOR_THRESH = 330;                        //threshold for colour sensor, >COLOR_THRESH implies object is directly ahead


	private UltrasonicSensor bottomUs;
	private ColorSensor colorSens;
	private ObjectRecognition recog;
	private PathGenerator pathGenerator;
	private Odometer odometer;

	private int role = 1;                //robots role
	private int avoidCount = 0;
	private double safeX = 0.0;
	private double safeY = 0.0;

	private int towerHeight = 0;
	private int numTowers = 0;        
	private double LW_RADIUS;
	private double RW_RADIUS;
	private double WHEEL_BASE;

	public Boolean isBusy = true;                        //either inspecting or turning
	public Boolean hasBlock = false;                //true once a block is read and grabbed
	public Boolean resetPath = false;                //true once a new path needs to be implemented
	public Boolean recentlyAvoided = false;
	public Boolean leftAvoidFail = false;
	public Boolean rightAvoidFail = false;
	public Boolean obstacleInWay = false;

	private double gx0=120;                        //green zone left x component
	private double gx1=150;                        //green zone right x component
	private double gy0=90;                        //green zone lower y component
	private double gy1=150;                        //green zone upper y component

	private double rx0;                        //red zone left x component
	private double rx1;                        //red zone right x component
	private double ry0;                        //red zone lower y component
	private double ry1;                        //red zone upper y component

	private double xDeposit = gx0 + 15.0;
	private double xDeposit2 = xDeposit;
	private double xDeposit3 = xDeposit;
	private double yDeposit = gy0;
	private double yDeposit2 = yDeposit;
	private double yDeposit3 = yDeposit;
	private double depositAngle = 90.0;

	private double wx0 = -30.0;                        //left wall
	private double wx1 = 210.0;                        //right wall
	private double wy0 = -30.0;                        //lower wall
	private double wy1 = 210.0;                        //upper wall

	private double[] xPath = new double[40];        //the path of x coordinates to travel to
	private double[] yPath = new double[40];        //the path of y coordinate to travel to, should always be synched with xPath
	private int pathIndex = 0;                                //used to synchronize x and y path arrays

	private double xDestination = 0.0;                //the desired destination at the end of the current path
	private double yDestination = 0.0;        
	
	private double startingX = 0.0;
	private double startingY = 0.0;

	int randX;
	int randY;
	double previousX;
	double previousY;

	// test
	private BTSend bts;                        //Bluetooth sender class


	/**
	 * Constructor
	 * 
	 * @param odom The shared odometer
	 * @param sender The bluetooth class that sends signals to the master brick
	 * @param bot The bottom ultrasonic sensor
	 * @param cs The color sensor used for object detection and recognition
	 * @param or The object recognition class that computes light readings
	 */
	public Navigation(Odometer odom, BTSend sender, UltrasonicSensor bot, ColorSensor cs, ObjectRecognition or, PathGenerator pg) {
		bottomUs = bot;
		odometer = odom;
		bts = sender;
		colorSens = cs;
		recog = or;
		pathGenerator = pg;
		LW_RADIUS = odometer.getLeftRadius();        //update wheel values
		RW_RADIUS = odometer.getRightRadius();
		WHEEL_BASE = odometer.getWheelBase();
		sensMotor.resetTachoCount();// straight is 0.0, left is -80, right is +80
	}


	/**
	 * Controls the flow of execution in the Navigation class
	 * 
	 * @return void
	 */
	public void run() {
		calculateDepositPoint();
		randomPathFinder();


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
	public void travelTo(double x, double y, Boolean ignore){ 

		isBusy = false;                 //odometer should be corrected while traveling, only time where robot is not busy is when travelling
		boolean first = true;                 //true only for first iteration of while loop, turn to minAng with greater accuracy
		double minAng;
		double masterAng = 0;                //the desired angle the robot should be traveling at
		colorSens.setFloodlight(Color.RED);                //turn on floodlight for obstacle detection
		double distance = Math.sqrt(Math.pow(Math.abs(x -odometer.getX()), 2) + Math.pow(Math.abs(y - odometer.getY()), 2));        //pythagorean theorum        
		double masterDist = distance;                //distance the robot will have to travel
		//rotateSensorsLeft(11, true);

		while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {

			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0/ Math.PI);         //update minimum angle
			if (minAng < 0) minAng += 360.0;

			distance = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()), 2));        //update distance traveled

			LCD.drawString("d: " + odometer.getMasterDistance(), 0, 4, false);

			//object detected immediately in front of robot
			if(colorSens.getNormalizedLightValue() > COLOR_THRESH - 10){
				this.setSpeeds(0,0);
				//begin inspection/avoidance, robot is busy
				isBusy = true;

				//check the color
				if((!hasBlock) && (!ignore)){
					if(checkBlockColor()){
						//capture the block
						capture();        
						return;
					}else{
						obstacleInWay = true;
						leftMotor.backward();
						rightMotor.backward();
						leftMotor.setSpeed(SLOW);
						rightMotor.setSpeed(SLOW);
						leftMotor.rotate(-convertDistance(LW_RADIUS, 8.0), true);
						rightMotor.rotate(-convertDistance(RW_RADIUS, 8.0), false);
						leftMotor.stop();
						rightMotor.stop();
						return;
					}
				}else{
					if(!checkBlockColor()){
						recentlyAvoided = true;
						avoid();
						return;
					}else{
						rotateSensorsRight(80, false);
						leftMotor.backward();
						rightMotor.backward();
						leftMotor.setSpeed(FAST);
						rightMotor.setSpeed(FAST);
						leftMotor.rotate(convertDistance(LW_RADIUS, 7.0), true);
						rightMotor.rotate(convertDistance(RW_RADIUS, 7.0), false);
						rotateSensorsLeft(80, false);
					}
					//both capture and avoid will move the robot past it's next destination, break this travelTo call
					//return;
				}
			}

			//Only turn with a small threshold for the first iteration, otherwise the robot is too oscillatory
			if (first){ 
				this.turnTo(minAng, true, true);
				masterAng = minAng;
				first = false;
			}

			//Find error in heading
			double angleError = minAng - odometer.getTheta();
			if(angleError > 180.0){
				angleError = angleError - 360.0;
			}
			else if(angleError < -180.0){
				angleError = angleError + 360.0;
			}

			if(Math.abs(angleError) > 5){ 
				this.turnTo(minAng,true, false);
			}
			else{
				this.setSpeeds(FAST - (int)(2.0 * angleError), FAST + (int)(2.0 * angleError));
			}

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
		double endAngle = odometer.getTheta() - 89.0;        //set end angle of scan
		colorSens.setFloodlight(Color.RED);
		if(endAngle < 0){
			endAngle += 360;
		}
		turnTo(odometer.getTheta() + 90.0, true, true);        //turn to start angle of scan

		try {Thread.sleep(500);} catch (InterruptedException e) {}                //wait for half a second so ultrasonic sesnor can stabilize

		//start rotating
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.backward();

		int dist;                                                        //distance read from us sensor
		Boolean object = false;                                //true when scan is passing over an object, false in between
		int objDist = 0;                                        //previously read distance of object

		int[][] objects = new int[4][20];        //assuming 20 is the max objects the robot can distinguish in one square
		int objIndex = 0;        //synchronizes the object distance and angle in the 2D array


		while (Math.abs(odometer.getTheta() - endAngle) > 2) {
			dist = getFilteredDistance();                //update the distance


			//if the reading is within a 32cm radius
			if (dist < 32) {
				if (object == false) {
					// first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;                //store the distance of the first edge
					objects[1][objIndex] = (int) odometer.getTheta();                //store the angle of the first edge
					Sound.beep();
				} else {
					// still reading the same object, update the object distance
					objDist = dist;
				}
			}
			LCD.drawString("HEREEE:" + endAngle, 0, 5, false);
			//if there is a difference >9cm between the previously read distance and the new distance, it is the end of an object
			if ((Math.abs(objDist - dist) > 9) && (object == true)) {
				//end of object, all data known for that object
				objects[2][objIndex] = objDist;
				objects[3][objIndex] = (int) odometer.getTheta();
				objIndex++;
				object = false;
				Sound.beep();
			}

			if(colorSens.getNormalizedLightValue() > COLOR_THRESH - 10){
				leftMotor.stop();
				rightMotor.stop();
				if(checkBlockColor()){
					capture();
					return true;
				}else{
					leftMotor.setSpeed(SLOW);
					rightMotor.setSpeed(SLOW);
					leftMotor.forward();
					rightMotor.backward();
				}
			}


		}
		this.setSpeeds(0,0);
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

			//handle odometer rollover from 0 to 359
			if((objects[1][i] <= 90.0) && (objects[3][i] >= 270)){
				objects[3][i] -= 360.0;
			}

			//angle to travel at
			travelAng = ((objects[1][i] + objects[3][i]) / 2.0 + ANGLE_OFFSET); 

			//checking for bad distance readings
			if ((travelDist > 0) && (pathGenerator.checkPointAhead(travelAng, travelDist))){

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
		if(!checkBlockColor()){
			//all checks read object as wood block, return sensor to starting position


			//reverse robot to starting position of inspection
			leftMotor.rotate(-convertDistance(LW_RADIUS, distTraveled), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, distTraveled), false);
			leftMotor.stop();
			rightMotor.stop();
			return false;
		}else{
			capture();
			return true;
		}
	}

	/**
	 * Assumes a blue block is in front of the robot, will reverse, lower claw, and grab and lift the block
	 */
	public void capture(){
		double distance = 20.0;

		//reverse far enough to lower claw
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();

		//move sensors out of the way of the claw
		rotateSensorsRight(80, false);

		//send signal from BTSend class to lower arms and open the claw (this signal is 1)
		try {bts.sendSignal(1);} catch (IOException e) {}        

		//wait for claw to come down
		try {Thread.sleep(2500);} catch (InterruptedException e1) {}

		//travel back to the block and slightly farther to position block in claw
		leftMotor.rotate(convertDistance(LW_RADIUS, distance+5), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, distance+5), false);



		leftMotor.stop();
		rightMotor.stop();

		try {bts.sendSignal(3);} catch (IOException e) {Sound.buzz();}
		try {Thread.sleep(1000);} catch (InterruptedException e1) {}

		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, 12), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 12), false);
		leftMotor.stop();
		rightMotor.stop();

		try {bts.sendSignal(2);} catch (IOException e) {Sound.buzz();}        //2 for raise arms
		//try {bts.sendSignal(3);} catch (IOException e) {Sound.buzz();}        //3 for clamping only

		//wait for lift and clamp
		try {Thread.sleep(1700);} catch (InterruptedException e1) {}

		//return sensors to original orientation
		rotateSensorsLeft(80, false);

		//drive and deposit in green zone.
		finishLine();
	}

	/**
	 * Will take the robot from it's current position directly to the bottom left corner of the green zone
	 */
	public void finishLine(){
		hasBlock = true;
		//rotate sensors away from claw


		travelToDepositPoint();
		rotateSensorsRight(80, false);

		if(role == 1){
			//send signal to lower arms all the way and open claw - MAY NEED TO BE CHANGED FOR TOWER BUILDING
			if(towerHeight == 0){

				try {bts.sendSignal(1);} catch (IOException e) {}
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 20), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 20), false);

				try {bts.sendSignal(3);} catch (IOException e) {}        //clamp claw - claw must be clamped to be lifted
				try {Thread.sleep(500);} catch (InterruptedException e1) {}
				try {bts.sendSignal(2);} catch (IOException e) {}        //raise arms to max
				try {Thread.sleep(2000);} catch (InterruptedException e1) {}

			}else if (towerHeight == 1){
				try {bts.sendSignal(10);} catch (IOException e) {}                //command for one block tower height
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				//reverse far enough to raise claw to the top again
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 20), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 20), false);

				try {bts.sendSignal(-10);} catch (IOException e) {}
				try {Thread.sleep(1000);} catch (InterruptedException e1) {}
			}else if (towerHeight == 2){
				try {bts.sendSignal(11);} catch (IOException e) {}                //command for two block tower height
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				//reverse far enough to raise claw to the top again
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 15), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 15), false);

				try {bts.sendSignal(12);} catch (IOException e) {}        
				try {Thread.sleep(1500);} catch (InterruptedException e1) {}
				
				travelToStartingPoint();
				
				//for tower pushing
				/*try {bts.sendSignal(-11);} catch (IOException e) {}                        //bring claw to floor, clamp, move forward to push tower in
				try {Thread.sleep(1000);} catch (InterruptedException e1) {}

				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.forward();
				rightMotor.forward();
				leftMotor.rotate(convertDistance(LW_RADIUS, 30), true);
				rightMotor.rotate(convertDistance(RW_RADIUS, 30), false);

				//reverse far enough to raise claw to the top again
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 30), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 30), false);


				try {bts.sendSignal(4);} catch (IOException e) {}
				try {Thread.sleep(1000);} catch (InterruptedException e1) {}*/
			}
			

		}else{
			try {bts.sendSignal(1);} catch (IOException e) {}
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}

			leftMotor.setSpeed(SLOW);
			rightMotor.setSpeed(SLOW);
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.rotate(-convertDistance(LW_RADIUS, 25), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, 25), false);

			try {bts.sendSignal(4);} catch (IOException e) {}
			try {Thread.sleep(1000);} catch (InterruptedException e1) {}
			
			travelToStartingPoint();
			
		}

		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, 10), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 10), false);

		if(role == 1){
			towerHeight++;
		}

		if(towerHeight == 3){
			towerHeight = 0;
			numTowers++;
		}

		rotateSensorsLeft(80, false);
		hasBlock = false;



		//will not scan and will generate a new point behind the green zone

		obstacleInWay = true;
	}
	
	public void travelToStartingPoint(){
		double [][] corners = new double [4][2];
		corners = pathGenerator.findClosestCorner();
		boolean clearGreen;

		clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), startingX, startingY);
		if (!clearGreen) {
			for (int i = 0; i < corners.length; i++) {
				travelTo(corners[i][0], corners[i][1], false);
				while(recentlyAvoided){
					if(avoidCount == 3){
						avoidCount = 0;
						break;
					}
					recentlyAvoided = false;
					avoidCount++;
					travelTo(corners[i][0], corners[i][1], false);
				}
				clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), startingX, startingY);
				if (clearGreen) {
					travelTo(startingX, startingY, false);
					break;
				}	
			}
		}
		else {
			travelTo(startingX, startingY, false);
			while(recentlyAvoided){
				if(avoidCount == 3){
					avoidCount = 0;
					break;
				}
				recentlyAvoided = false;
				avoidCount++;
				travelTo(startingX, startingY, false);
			}
			
		}
		
		while(true){
			LCD.drawString("DEMO COMPLETE", 0, 4, false);
		}
	}

	public void travelToDepositPoint(){
		double x = odometer.getX();
		double y = odometer.getY();
		double [][] corners = new double [4][2];
		corners = pathGenerator.findClosestCorner();
		double[] borderPoint = new double[2];
		boolean clearGreen;
		borderPoint = pathGenerator.calculateBorderPoint();

		if(role == 1){
			borderPoint = pathGenerator.calculateBorderPoint();
			clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
			if (!clearGreen) {
				for (int i = 0; i < corners.length; i++) {
					travelTo(corners[i][0], corners[i][1], false);
					while(recentlyAvoided){
						if(avoidCount == 3){
							avoidCount = 0;
							break;
						}
						recentlyAvoided = false;
						avoidCount++;
						travelTo(corners[i][0], corners[i][1], false);
					}
					clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
					if (clearGreen) {
						travelTo(borderPoint[0], borderPoint[1], false);
						break;
					}	
				}
			}
			else {
				travelTo(borderPoint[0], borderPoint[1], false);
			}

			circleGreenZone(xDeposit, yDeposit);
			safeX = odometer.getX();
			safeY = odometer.getY();

		}
		else{
			borderPoint = pathGenerator.calculateBorderPoint();
			clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
			if (!clearGreen) {
				for (int i = 0; i < corners.length; i++) {	
					travelTo(corners[i][0], corners[i][1], false);
					while(recentlyAvoided){
						if(avoidCount == 3){
							avoidCount = 0;
							break;
						}
						recentlyAvoided = false;
						avoidCount++;
						travelTo(corners[i][0], corners[i][1], false);
					}
					clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
					if (clearGreen) {                                                       
						travelTo(borderPoint[0], borderPoint[1], false);
						break;
					}
				}
			}
			else{
				travelTo(xDeposit, yDeposit, false);
			}
		}
		turnTo(depositAngle, true, true);
	}

	//once at a border point, drives straight at 0, 90, 180, or 270 around the green zone to get to the deposit point
	public void circleGreenZone(double xDest, double yDest){
		double heading = Math.atan2(yDest - odometer.getY(), xDest - odometer.getX()) * (180.0/ Math.PI);

		if(heading <= 90.0 && heading > 0.0){
			travelTo(odometer.getX(), yDest+10, true);
			travelTo(xDest, yDest + 10, true);
			travelTo(xDest, yDest, true);
		}else if (heading <= 180.0 && heading > 90){
			travelTo(xDest-10, odometer.getY(), true);
			travelTo(xDest-10, yDest, true);
			travelTo(xDest, yDest, true);
		}else if (heading <= 270.0 && heading > 180.0){
			travelTo(odometer.getX(), yDest-10, true);
			travelTo(xDest, yDest-10, true);
			travelTo(xDest, yDest, true);
		}else{
			travelTo(xDest+10, odometer.getY(), true);
			travelTo(xDest+10, yDest, true);
			travelTo(xDest, yDest, true);
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

			if(wallFollowLeft(initAngle)){
				leftAvoidFail = false;
			}else{
				leftAvoidFail = true;
			}


			/*if(leftAvoidFail){
                                turnTo(odometer.getTheta() - 180.0, true, true);
                                wallFollowRight(initAngle);
                                if(!rightAvoidFail){
                                        leftAvoidFail = false;
                                }
                        }*/

		}else{
			//turn to the right
			turnTo(odometer.getTheta() - 180.0, true, true);
			dist = getFilteredDistance();

			//check the distance to the right
			if(dist > 30){
				//nothing within 30cm, avoid to the right
				if(wallFollowRight(initAngle)){
					rightAvoidFail = false;
				}else{
					rightAvoidFail = true;
				}

				/*if(rightAvoidFail){
                                        turnTo(odometer.getTheta() - 180.0, true, true);

                                        rotateSensorsRight(40, false);
                                        wallFollowLeft(initAngle);
                                        if(!leftAvoidFail){
                                                rightAvoidFail = false;
                                        }
                                        rotateSensorsLeft(40, false);
                                }*/

			}else{
				//turn to initial angle and reverse 30cm backward
				turnTo(initAngle-180, true, true);
				leftMotor.setSpeed(FAST);
				rightMotor.setSpeed(FAST);
				leftMotor.forward();
				rightMotor.forward();

				if(wallFollowLeft(initAngle)){
					leftAvoidFail = false;
				}else{
					leftAvoidFail = true;
				}
				try {Thread.sleep(1000);} catch (InterruptedException e) {}
			}
		}

		if(leftAvoidFail && rightAvoidFail){
			leftMotor.stop();
			rightMotor.stop();

		}else if (leftAvoidFail){
			turnTo(odometer.getTheta() - 180.0, true, true);
			wallFollowRight(initAngle);
		}else if (rightAvoidFail){
			turnTo(odometer.getTheta() + 180.0, true, true);
			wallFollowLeft(initAngle);
		}

		//if not carrying a block, regenerate path to green zone
		/*if(hasBlock){
                        finishLine();
                }*/

	}

	public Boolean wallFollowLeft(double initAngle){
		rotateSensorsRight(40, false);
		double endAngle = initAngle - 20;
		if(endAngle < 0){
			endAngle += 360.0;
		}
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

			if (!pathGenerator.checkPointAhead(odometer.getTheta(), 10)){
				leftAvoidFail = true;
				rotateSensorsLeft(40, false);
				return false;
			}


			//cap the error so the speed doesn't go crazy
			if(error > 100){
				error = 50;
			}else if (error < -100){
				error = -50;
			}

			if(error < -BAND_WIDTH){
				//too far away, need to turn right
				leftMotor.setSpeed(FAST + 75);
				rightMotor.setSpeed(FAST + 60 + (error*2));        //error is negative, right move slower
			}else if (error > BAND_WIDTH){
				//too close, need to turn left
				leftMotor.setSpeed(FAST + 75);
				rightMotor.setSpeed(FAST + 60 + (error * 2));        //error is positive, right moves faster
			}else{
				leftMotor.setSpeed(FAST + 100);
				rightMotor.setSpeed(FAST + 100);
			}

			//if the robot's orientation reaches its initial angle - 20 degrees, wall is assumed to be passed
			if (Math.abs(odometer.getTheta() - endAngle) < ANGLE_THRESH){
				Sound.buzz();
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				break;
			}
		}

		rotateSensorsLeft(40, false);

		//drive straight for 1.5s to get sufficiently apst the block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}

		return true;
	}

	public Boolean wallFollowRight(double initAngle){

		rotateSensorsLeft(40, false);

		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();

		double endAngle = (initAngle + 20.0)%360.0;
		int BAND_CENTER = 20;
		int BAND_WIDTH = 2;
		int dist;
		int error;
		//actual wall following
		while(true){
			dist = getFilteredDistance();
			error = BAND_CENTER - dist;

			if (!pathGenerator.checkPointAhead(odometer.getTheta(), 10)){
				rotateSensorsRight(40, false);
				rightAvoidFail = true;
				return false;
			}

			if(error > 100){
				error = 50;
			}else if (error < -100){
				error = -50;
			}
			//if robot gets to starting x orientation, it has successfully avoided the block

			if(error < -BAND_WIDTH){
				//too far away, need to turn left
				leftMotor.setSpeed(FAST + 60 + (error*2));        //error is negative, left moves slower
				rightMotor.setSpeed(FAST + 75);        
			}else if (error > BAND_WIDTH){
				//too close, need to turn right
				leftMotor.setSpeed(FAST + 60 + (error*2));        //error is positive, left moves faster
				rightMotor.setSpeed(FAST + 75);
			}else{
				//within band, go straight
				leftMotor.setSpeed(FAST + 100);
				rightMotor.setSpeed(FAST + 100);
			}

			//if it reaches int's initial angle + 20 degrees, assumed avoided object
			if (Math.abs(odometer.getTheta() - endAngle) < ANGLE_THRESH){
				Sound.buzz();
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				break;
			}

		}

		rotateSensorsRight(40, false);

		//drive forward to get past block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}

		return true;

	}


	/**
	 * Filter for ultrasonic readings, for rotating sensor
	 * 
	 * @return The filtered distance reading
	 */
	public int getFilteredDistance() {
		int distance = 0;
		bottomUs.ping();
		if (bottomUs.getDistance() > 150) {
			distance = 150;
		}
		distance = bottomUs.getDistance();
		return distance;
	}


	/**
	 * Rotates sensor motor clockwise 80 degrees
	 */
	public void rotateSensorsRight(int angle, boolean ret){
		sensMotor.setSpeed(150);
		sensMotor.forward();
		sensMotor.rotate(angle, ret);
		sensMotor.stop();
	}

	/**
	 * Rotates sensor motor counter-clockwise 80 degrees
	 */
	public void rotateSensorsLeft(int angle, boolean ret){
		sensMotor.setSpeed(150);
		sensMotor.backward();
		sensMotor.rotate(-angle, ret);
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

	public void setTransmission(int[] green, int[] red, int role, double x, double y){
		pathGenerator.setZones(green, red);
		startingX = x;
		startingY = y;

		this.gx0 = (double)green[0] * 30.0;
		this.gy0 = (double)green[1] * 30.0;
		this.gx1 = (double)green[2] * 30.0;
		this.gy1 = (double)green[3] * 30.0;

		this.rx0 = (double)red[0] * 30.0;
		this.ry0 = (double)red[1] * 30.0;
		this.rx1 = (double)red[2] * 30.0;
		this.ry1 = (double)red[3] * 30.0;

		this.role = role;

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
		while(true){
			randX = x.nextInt(220);
			randY = x.nextInt(220);

			if(hasBlock){
				finishLine();
				continue;
			}

			LCD.drawString("Generating points", 0, 4, false);

			//no obstacle avoidance necessary, get new point "behind" the robot
			if(obstacleInWay){
				double heading = Math.atan2(randY - odometer.getY(), randX - odometer.getX()) * (180.0/ Math.PI);
				if(Math.abs(heading - odometer.getTheta()) <= 90){
					continue;
				}else{
					//accept these values, no longer obstacle in way
					obstacleInWay = false;
				}
			}

			double distToPoint = Math.sqrt(Math.pow(Math.abs(odometer.getX() - randX), 2) + Math.pow(Math.abs(odometer.getY() - randY), 2));
			if((distToPoint > 100) || (distToPoint < 10)){
				continue;
			}

			//If destination coordinate is in red zone, green zone, or within the wall threshold, 
			//continue and generate new points
			if (!pathGenerator.checkPoint((double)randX, (double)randY, 10.0)){
				continue;
			}

			goodPath = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), randX, randY);
			if (goodPath) {
				travelTo(randX, randY, false);
				if(!obstacleInWay){
					scan();
				}
			}
		} 
	}


	public void calculateDepositPoint(){
		if(role == 1){
			if ((gx1 - gx0) == 30.0){
				xDeposit = (gx1 + gx0)/2.0;
				xDeposit2 = xDeposit;
				xDeposit3 = xDeposit;
				if(gy0 >= (wy1/2.0)){
					yDeposit = gy0;
					yDeposit2 = yDeposit - 7.6;
					yDeposit3 = yDeposit - 9.0;
					depositAngle = 90.0;
				}else{
					yDeposit = gy1;
					yDeposit2 = yDeposit + 7.6;
					yDeposit3 = yDeposit + 9.0;
					depositAngle = 270.0;
				}
			}else{
				yDeposit = (gy1 + gy0)/2.0;
				yDeposit2 = yDeposit;
				yDeposit3 = yDeposit;

				if(gx0 >= (wx1/2.0)){
					xDeposit = gx0;
					xDeposit2 = xDeposit - 7.6;
					xDeposit3 = xDeposit - 9.0;
					depositAngle = 0.0;
				}else{
					xDeposit = gx1;
					xDeposit2 = xDeposit + 7.6;
					xDeposit3 = xDeposit + 9.0;
					depositAngle = 180.0;
				}
			} 
		}else{
			if ((gx1 - gx0) == 60.0){
				depositAngle = 90.0;
			}else{
				depositAngle = 0.0;
			}
			xDeposit = (gx1 + gx0)/2.0;
			yDeposit = (gy1 + gy0)/2.0;
		}
	}

	public boolean checkBlockColor(){
		if(recog.checkColor()){
			return true;
		}else{
			rotateSensorsRight(15, false);
			if(recog.checkColor()){
				//if after the second reading, a blue block was sensed, return the sensor to it's initial position
				rotateSensorsLeft(15, false);        
				return true;
			}else{
				//first two sensor checks read wood blocks, rotate light sensor 30 degrees to the 
				//left (15 degrees from starting orientation)
				rotateSensorsLeft(30, false);
				if(recog.checkColor()){

					//rotate sensor back to initial position
					rotateSensorsRight(15, false);
					return true;
				}
			}
		}
		rotateSensorsRight(15, false);
		return false;
	}




}