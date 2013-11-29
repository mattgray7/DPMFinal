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
	private BTSend bts;                        	//Bluetooth sender class


	private int role = 1;                		//robots role
	private int avoidCount = 0;
	private double safeX = 0.0;
	private double safeY = 0.0;

	private int towerHeight = 0;
	private int numTowers = 0;  
	private int numDrops = 0;
	private double LW_RADIUS;
	private double RW_RADIUS;

	public Boolean isBusy = true;                   //either inspecting or turning
	public Boolean hasBlock = false;                //true once a block is read and grabbed
	public Boolean recentlyAvoided = false;
	public Boolean leftAvoidFail = false;
	public Boolean rightAvoidFail = false;
	public Boolean obstacleInWay = false;
	public Boolean drivingHome = false;

	private double gx0=60;                        //green zone left x component
	private double gx1=90;                        //green zone right x component
	private double gy0=60;                        //green zone lower y component
	private double gy1=90;                        //green zone upper y component

	private double rx0;                        //red zone left x component
	private double rx1;                        //red zone right x component
	private double ry0;                        //red zone lower y component
	private double ry1;                        //red zone upper y component

	//3 deposit coordinates for different positioning when building a 3 block tower
	private double xDeposit = gx0 + 15.0;		
	private double xDeposit2 = xDeposit;
	private double xDeposit3 = xDeposit;
	private double yDeposit = gy0;
	private double yDeposit2 = yDeposit;
	private double yDeposit3 = yDeposit;
	private double depositAngle = 90.0;

	private double wx0 = -30.0;                        //left wall
	private double wx1 = 330.0;                        //right wall
	private double wy0 = -30.0;                        //lower wall
	private double wy1 = 330.0;                        //upper wall

	private double startingX = 0.0;				//starting x coordinate
	private double startingY = 0.0;				//starting y coordinate

	private int randX;
	private int randY;

	

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
		sensMotor.resetTachoCount();				// straight is 0.0, left is -80, right is +80
	}


	/**
	 * Controls the flow of execution in the Navigation class
	 * 
	 * @return void
	 */
	public void run() {
		//calculate the optimal point to drop blocks depending on zone location and orientation
		calculateDepositPoint();
		randomPathFinder();
	}
	
	
	/**
	 * Generates random coordinates that are not too close or far from the robot, and are not located
	 * in danger zones.
	 */
	public void randomPathFinder() {
		boolean goodPath;
		Random x = new Random();
		
		//never finishes
		while(true){
			
			//even though walls are from -30 to 330, dont want to generate a point close to a wall
			randX = x.nextInt(280);
			randY = x.nextInt(280);

			//don't go to random point with block, travel to deposit zone
			if(hasBlock){
				finishLine();
				continue;
			}

			LCD.drawString("Generating points", 0, 4, false);

			//no obstacle avoidance necessary, get new point "behind" the robot
			if(obstacleInWay){
				double heading = Math.atan2(randY - odometer.getY(), randX - odometer.getX()) * (180.0/ Math.PI);
				if(Math.abs(heading - odometer.getTheta()) <= 90){
					//new point is "ahead of the robot, disregard the point
					continue;
				}else{
					//accept these values, no longer obstacle in way
					obstacleInWay = false;
				}
			}

			double distToPoint = Math.sqrt(Math.pow(Math.abs(odometer.getX() - randX), 2) + Math.pow(Math.abs(odometer.getY() - randY), 2));
			if((distToPoint > 75) || (distToPoint < 10)){
				//to ensure the robot does not travel for very long without scanning, distance from point is capped
				continue;
			}

			//If destination coordinate is in red zone, green zone, or within the wall threshold, 
			//continue and generate new points
			if (!pathGenerator.checkPoint((double)randX, (double)randY, 15.0)){
				continue;
			}

			goodPath = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), randX, randY);
			if (goodPath) {
				//the path is clear to the zone, travel there
				travelTo(randX, randY, false);
				
				//as long as no obstacle is sensed once travelTo is returned, do a 180 degree scan
				if(!obstacleInWay){
					scan();
				}
			}
		} 
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
		
		while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {

			//update minimum angle
			minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0/ Math.PI);         
			if (minAng < 0) minAng += 360.0;

			//object detected immediately in front of robot
			if((colorSens.getNormalizedLightValue() > COLOR_THRESH - 10) && (!ignore)){
				this.setSpeeds(0,0);
				
				//begin inspection/avoidance, robot is busy
				isBusy = true;

				//check the color
				if(!hasBlock){
					if(checkBlockColor()){
						//capture the block
						if(!drivingHome){
							//only capture if you are not trying to reach starting point again
							capture();        
							return;
						}
					}else{
						//boolean informs random point generator to choose a point "behind the robot" so as not to travel into the same obstacle
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
					if(checkBlockColor()){
						//when carrying a block, another blue block should not affect odometer. As such, no avoidance is necessary
						//but the blue block needs to be moved from the front of the sensor. The following code rotates the sensor
						//to one side, travel closer to the block, and rotates the sensor back knocking the blue block out of the way
						
						rotateSensorsRight(80, false);
						leftMotor.forward();
						rightMotor.forward();
						leftMotor.setSpeed(FAST);
						rightMotor.setSpeed(FAST);
						leftMotor.rotate(convertDistance(LW_RADIUS, 8.0), true);
						rightMotor.rotate(convertDistance(RW_RADIUS, 8.0), false);
						
						sensMotor.setSpeed(450);
						sensMotor.backward();
						sensMotor.rotate(-80, false);
						sensMotor.stop();
						
						leftMotor.backward();
						rightMotor.backward();
						leftMotor.setSpeed(FAST);
						rightMotor.setSpeed(FAST);
						leftMotor.rotate(-convertDistance(LW_RADIUS, 8.0), true);
						rightMotor.rotate(-convertDistance(RW_RADIUS, 8.0), false);
					}else{
						//only need to avoid when carrying a block to a zone
						recentlyAvoided = true;
						avoid(x, y);
						return;
					}
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
			
			//if error is large enough, stop and correct with weaker precision
			if(Math.abs(angleError) > ANGLE_THRESH){ 
				this.turnTo(minAng,true, false);
			}
			else{
				//otherwise do a smooth correction
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

		//stop the spinning
		if (stop) { 
			this.setSpeeds(0, 0); 
		} 

		//no longer turning, so now longer busy
		isBusy = false; 
	}


	/**
	 * Will stop and scan 180 degrees clockwise in an attempt to detect objects
	 * 
	 * @return true If a blue block was found
	 * @return false If no blue block was found
	 */
	public Boolean scan() {
		//set end angle of scan
		double endAngle = odometer.getTheta() - 90.0;      
		
		//account for odometer rolling over from 0 to 359
		if(endAngle < 0){
			endAngle += 360;
		}
		colorSens.setFloodlight(Color.RED);

		//turn to start angle of scan
		turnTo(odometer.getTheta() + 90.0, true, true);        

		//wait for half a second so ultrasonic sesnor can stabilize

		try {Thread.sleep(500);} catch (InterruptedException e) {}                
		
		//start rotating
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.backward();

		int dist;                                              	//distance read from us sensor
		Boolean object = false;                                	//true when scan is passing over an object, false in between objects
		int objDist = 0;                                       	//previously read distance of object

		int[][] objects = new int[4][20];        				//assuming 20 is the max edges the robot can distinguish in one scan
		int objIndex = 0;        								//synchronizes the object distance and angle in the 2D array


		while (Math.abs(odometer.getTheta() - endAngle) > 2) {
			//update the distance
			dist = getFilteredDistance();                

			//if the reading is within a 32cm radius
			if (dist < 32) {
				if (object == false) {
					// first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;                					//store the distance of the first edge
					objects[1][objIndex] = (int) odometer.getTheta();               //store the angle of the first edge
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
			}
			
			//if during the scan, the light sensor reads an object directly in front of it, stop and check it's color
			if(colorSens.getNormalizedLightValue() > COLOR_THRESH - 10){
				leftMotor.stop();
				rightMotor.stop();
				if(checkBlockColor()){
					//abandon the scan and capture the blue block immediately
					capture();
					return true;
				}else{
					//wood block, continue scanning
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

		// if scan ends without reading the falling edge of the object, set it manually
		if (object) {
			objects[2][objIndex] = objDist;
			objects[3][objIndex] = (int) odometer.getTheta();
		}

		int travelDist = 0;
		double travelAng = 0;
		double ANGLE_OFFSET;

		//for each object read
		for (int i = 0; i <= objIndex; i++) {
			
			//distance is the average of the distances read at each edge
			double avgDist = (objects[0][i] + objects[2][i]) / 2.0;
			travelDist = (int)avgDist- 10; 			//-10 to compensate for sensor inaccuracy

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

			//handle odometer rollover from 0 to 359 if the first angle was in the first quadrant and the 
			//second was in the fourth quadrant
			if((objects[1][i] <= 90.0) && (objects[3][i] >= 270)){
				objects[3][i] -= 360.0;
			}

			//angle to travel at
			travelAng = ((objects[1][i] + objects[3][i]) / 2.0 + ANGLE_OFFSET); 

			//checking for bad distance readings and if the inspection point is in the danger zone
			if ((travelDist > 0) && (pathGenerator.checkPointAhead(travelAng, travelDist))){

				colorSens.setFloodlight(Color.RED);
				boolean found = inspect(travelAng, travelDist);			//investigate each object
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
	 * @return True if a blue block was detected and captured
	 * @return False if no blue block was found
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
		
		//drive until the light sensor reads an object or until distance has been traveled
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


		//check the color
		if(!checkBlockColor()){
			//all 3 checks with light sensor read object as wood block, assume inspection failure and return

			//reverse robot to starting position of inspection
			leftMotor.rotate(-convertDistance(LW_RADIUS, distTraveled), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, distTraveled), false);
			leftMotor.stop();
			rightMotor.stop();
			return false;
		}else{
			//grab blue block and return method
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

		//clamps the claw closed
		try {bts.sendSignal(3);} catch (IOException e) {Sound.buzz();}
		try {Thread.sleep(1000);} catch (InterruptedException e1) {}
		
		//reverse before lifting arm as lifting has horizontal translation forward as well as vertical
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, 12), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 12), false);
		leftMotor.stop();
		rightMotor.stop();

		//raise arms to the maximum height with blcok in claw
		try {bts.sendSignal(2);} catch (IOException e) {Sound.buzz();}        //2 for raise arms

		//wait for lift
		try {Thread.sleep(1700);} catch (InterruptedException e1) {}

		//return sensors to original orientation
		rotateSensorsLeft(80, false);

		//drive and deposit in green zone.
		finishLine();
	}
	
	/**
	 * Pings the color sensor at three different angles to determine the color of the block
	 * @return True if a blue block was sensed
	 * @return False if no blue block was sensed
	 */
	public boolean checkBlockColor(){
		//Actual color check performed by ObjectRecognition class
		if(recog.checkColor()){
			return true;
		}else{
			//rotate sensor right and check again, this provides a more robust color scan
			rotateSensorsRight(15, false);
			if(recog.checkColor()){
				//if after the second reading, a blue block was sensed, return the sensor to it's initial position
				rotateSensorsLeft(15, false);        
				return true;
			}else{
				//first two sensor checks did not read a blue block, rotate light sensor 30 degrees to the 
				//left (15 degrees from starting orientation)
				rotateSensorsLeft(30, false);
				if(recog.checkColor()){
					//rotate sensor back to initial position
					rotateSensorsRight(15, false);
					return true;
				}
			}
		}
		
		//re-orient sensors, no blue block was found
		rotateSensorsRight(15, false);
		return false;
	}

	/**
	 * Will take the robot from it's current position directly to the deposit point of the green zone, or red zone for
	 * garbage collector
	 */
	public void finishLine(){
		hasBlock = true;

		travelToDepositPoint();
		
		//rotate sensors away from claw
		rotateSensorsRight(80, false);

		if(role == 1){
			//tower builder
			if(towerHeight == 0){

				//bring arm all the way down and open claw
				try {bts.sendSignal(1);} catch (IOException e) {}
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				//reverse so lifting arm does not move placed block
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 20), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 20), false);

				//clamp and raise claw to max height (claw must be clamed for arm lifting)
				try {bts.sendSignal(3);} catch (IOException e) {}        
				try {Thread.sleep(250);} catch (InterruptedException e1) {}
				try {bts.sendSignal(4);} catch (IOException e) {}        
				try {Thread.sleep(2000);} catch (InterruptedException e1) {}

			}else if (towerHeight == 1){
				//lower arm to height slightly above one block and open claw to drop block on top of the other
				try {bts.sendSignal(10);} catch (IOException e) {}                
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				//reverse far enough to raise claw to the top again
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 20), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 20), false);

				//raise arm to max height
				try {bts.sendSignal(-10);} catch (IOException e) {}
				try {Thread.sleep(1000);} catch (InterruptedException e1) {}
			}else if (towerHeight == 2){
				//lower arms to hieght slightly above two block tower height
				try {bts.sendSignal(11);} catch (IOException e) {}                
				try {Thread.sleep(2500);} catch (InterruptedException e1) {}

				//reverse far enough to raise claw to the top again
				leftMotor.setSpeed(SLOW);
				rightMotor.setSpeed(SLOW);
				leftMotor.backward();
				rightMotor.backward();
				leftMotor.rotate(-convertDistance(LW_RADIUS, 15), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 15), false);

				//raise claw to max height
				try {bts.sendSignal(12);} catch (IOException e) {}        
				try {Thread.sleep(1500);} catch (InterruptedException e1) {}
				
				

				/*
				//Lowers claw to floor and pushes 3 block tower further into the green zone to make room for second tower
				try {bts.sendSignal(-11);} catch (IOException e) {}                        
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
			//garbage collector - lower block to height above 2 block tower and release claw. This prevents arm 
			//from jamming/getting stuck when lowering block to floor if a block is already below it
			try {bts.sendSignal(11);} catch (IOException e) {}                
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}

			leftMotor.setSpeed(SLOW);
			rightMotor.setSpeed(SLOW);
			leftMotor.backward();
			rightMotor.backward();
			leftMotor.rotate(-convertDistance(LW_RADIUS, 25), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, 25), false);

			try {bts.sendSignal(12);} catch (IOException e) {}                //command for two block tower height
			try {Thread.sleep(2500);} catch (InterruptedException e1) {}

			
		}

		//reverse out of zone
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.rotate(-convertDistance(LW_RADIUS, 10), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 10), false);

		//increment block count
		if(role == 1){
			towerHeight++;
		}else{
			numDrops++;
		}

		if(towerHeight == 3){
			towerHeight = 0;
			numTowers++;
		}
		
		/* For competition, only have time to build one tower or drop 3 blocks
		if((numTowers == 1) || (numDrops == 3)){
			while(true){
				LCD.drawString("COMPLETED", 0, 4, false);
				LCD.drawString("(" + (int)xDeposit + ", " + (int)yDeposit + ")", 0, 5, false);
				LCD.drawString("(" + (int)xDeposit2 + ", " + (int)yDeposit2 + ")", 0, 6, false);
				LCD.drawString("(" + (int)xDeposit3 + ", " + (int)yDeposit3 + ")", 0, 7, false);
			}
		}*/

		//return sensors to normal position
		rotateSensorsLeft(80, false);
		hasBlock = false;

		//will not scan and will generate a new point behind the green zone
		obstacleInWay = true;
	}

	/**
	 * Drives the robot to the calculated deposit point.
	 */
	public void travelToDepositPoint(){
		//stores the corners of the red zone
		double [][] corners = new double [4][2];
		corners = pathGenerator.findClosestCorner();
		
		//stores the coordinate of the closest corner of the green zone
		double[] borderPoint = new double[2];
		borderPoint = pathGenerator.calculateBorderPoint();
		
		//true if path to green zone is unobstructed by red zone
		boolean clearGreen;

		if(role == 1){
			//tower builder
			borderPoint = pathGenerator.calculateBorderPoint();
			
			//check for path to green zone clarity
			clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
			
			if (!clearGreen) {
				//builder, path is not clear
				for (int i = 0; i < corners.length; i++) {
					
					//travel to each corner of red zone starting with the closest corner until the path to the green zone is clear
					travelTo(corners[i][0], corners[i][1], false);
					
					//if robot avoided while travlling to corner, try to reach that location again
					while(recentlyAvoided){
						
						//if this is the third attempt to go to the next point, abandon this point and travel to the next corner
						if(avoidCount == 2){
							avoidCount = 0;
							break;
						}
						recentlyAvoided = false;
						avoidCount++;
						if(pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1])){
							//if after avoiding, you are clear to go to green zone, travel there
							travelTo(borderPoint[0], borderPoint[1], false);
						}else{
							//if not, travel to the next corner
							travelTo(corners[i][0], corners[i][1], false);
						}
					}
					
					//if it got to the corner without avoiding, try and go to green zone again, else, go to next corner
					clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
					
					if (clearGreen) {
						travelTo(borderPoint[0], borderPoint[1], false);
						while(recentlyAvoided){
							if(avoidCount == 3){
								avoidCount = 0;
								break;
							}
							recentlyAvoided = false;
							avoidCount++;
							travelTo(borderPoint[0], borderPoint[1], false);
						}
						//once successfully at green zone, break corner loop
						break;
					}	
				}
			}else {
				//builder, path is clear, drive to green zone
				travelTo(borderPoint[0], borderPoint[1], false);
				while(recentlyAvoided){
					//keep trying to get to green zone after avoiding
					if(avoidCount == 3){
						avoidCount = 0;
						break;
					}
					recentlyAvoided = false;
					avoidCount++;
					travelTo(borderPoint[0], borderPoint[1], false);
				}
			}
			
			//once at green zone, circle green zone in a square-like fashion to reach each deposit point
			if(towerHeight == 0){
				circleGreenZone(xDeposit, yDeposit);
			}else if (towerHeight == 1){
				circleGreenZone(xDeposit2, yDeposit2);
			}else{
				circleGreenZone(xDeposit3, yDeposit2);
			}
			
			//the x and y coordinate of the robot before he drops the block
			safeX = odometer.getX();
			safeY = odometer.getY();

		}else{
			//garbage collector, path is not clear
			borderPoint = pathGenerator.calculateBorderPoint();
			clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
			
			if (!clearGreen) {
				//path isn't clear, travel to corners of restricted zone
				for (int i = 0; i < corners.length; i++) {	
					//travel to corners in an attempt to avoid zone
					travelTo(corners[i][0], corners[i][1], false);
					while(recentlyAvoided){
						if(avoidCount == 2){
							avoidCount = 0;
							break;
						}
						recentlyAvoided = false;
						avoidCount++;
						if(pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1])){
							//after avoidance, check for path to destination zone
							travelTo(borderPoint[0], borderPoint[1], false);
						}else{
							travelTo(corners[i][0], corners[i][1], false);
						}
					}
					
					//after travelling to corner, check for path clarity to destination zone
					clearGreen = pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), borderPoint[0], borderPoint[1]);
					if (clearGreen) {                                                       
						travelTo(borderPoint[0], borderPoint[1], false);
						while(recentlyAvoided){
							if(avoidCount == 2){
								avoidCount = 0;
								break;
							}
							recentlyAvoided = false;
							avoidCount++;
							travelTo(borderPoint[0], borderPoint[1], false);
						}
						break;
					}
				}
				
				//garbage collector, travel directly to deposit point as tower safety does not need to be considered
				travelTo(xDeposit, yDeposit, false);
				while(recentlyAvoided){
					if(avoidCount == 2){
						avoidCount = 0;
						break;
					}
					recentlyAvoided = false;
					avoidCount++;
					travelTo(xDeposit, yDeposit, false);
				}
				
			}else{
				//garbage collector is clear
				travelTo(xDeposit, yDeposit, false);
				while(recentlyAvoided){
					if(avoidCount == 2){
						avoidCount = 0;
						break;
					}
					recentlyAvoided = false;
					avoidCount++;
					travelTo(xDeposit, yDeposit, false);
				}
			}
		}
		
		//turn to the deposit angle, at this point, the robot is ready to deposit a block
		turnTo(depositAngle, true, true);
	}

	/**
	 * Once at a border point, drives straight at 0, 90, 180, or 270 around the green zone to get to the deposit point
	 * @param xDest X coordinate of deposit point
	 * @param yDest Y coordinate of deposit point
	 */
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
	 * Avoids an obstacle by wall following left or right depending on surrounding obstacles and zones
	 * @param xDest X coordinate of intended destination
	 * @param yDest Y coordinate of intended destination
	 */
	public void avoid(double xDest, double yDest) {
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
				
			}else{
				//turn around and wall follow left
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
			}
		}

		//AvoidFails are true when a danger zone is in that direction
		if(leftAvoidFail && rightAvoidFail){
			//rare case, surrounded by danger zone on each side
			leftMotor.stop();
			rightMotor.stop();
			if(pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), xDest, yDest)){
				//if possible, after last wall following attempt, travel to the green zone
				travelTo(xDest, yDest, false);
				return;
			}
			//otherwise, this method will return and a new point will be generated to travel to

		}else if (leftAvoidFail){
			if(pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), xDest, yDest)){
				//try going straight to the destination
				travelTo(xDest, yDest, false);
				return;
			}else{
				//otherwise, wall follow in the opposite direction
				turnTo(odometer.getTheta() - 180.0, true, true);
				wallFollowRight(initAngle);
			}
		}else if (rightAvoidFail){
			if(pathGenerator.checkPointsInPath(odometer.getX(), odometer.getY(), xDest, yDest)){
				//try and go straight to the destination
				travelTo(xDest, yDest, false);
				return;
			}else{
				//otherwise, wall follow in the opposite direction
				turnTo(odometer.getTheta() + 180.0, true, true);
				wallFollowLeft(initAngle);
			}
		}
	}

	/**
	 * Will perform p-type wall following around the left side of the obstacle
	 * @param initAngle The initial angle of the robot before wall following
	 * @return True if successful avoidance, false if danger zone is too close to safely avoid
	 */
	public Boolean wallFollowLeft(double initAngle){
		//rotate sensors to face the obstacle
		rotateSensorsRight(40, false);
		
		//once you reach this orientation again, assumed to be around obstacle
		double endAngle = initAngle - 20;
		
		//account for odometer roll over form 0 to 359
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

			//check if you are about to drive into a danger zone, if so, immediately stop and return left avoidance has failed
			if (!pathGenerator.checkPointAhead(odometer.getTheta(), 17)){
				leftAvoidFail = true;
				
				//re-orient sensors
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

		//re-orient sensors
		rotateSensorsLeft(40, false);

		//drive straight for 1.5s to get sufficiently past the block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}

		//successful avoidance
		return true;
	}

	/**
	 * Will perform p-type wall following around the right side of the obstacle
	 * @param initAngle The initial angle of the robot before wall following
	 * @return True if successful avoidance, false if danger zone is too close to safely avoid
	 */
	public Boolean wallFollowRight(double initAngle){

		//point sensors at obstacle
		rotateSensorsLeft(40, false);

		//start driving
		leftMotor.setSpeed(FAST);
		rightMotor.setSpeed(FAST);
		leftMotor.forward();
		rightMotor.forward();

		//destination angle - once reached, assumed to be successfully past object
		double endAngle = (initAngle + 20.0)%360.0;
		int BAND_CENTER = 20;
		int BAND_WIDTH = 2;
		int dist;
		int error;
		//actual wall following
		while(true){
			dist = getFilteredDistance();
			error = BAND_CENTER - dist;

			//if danger zone is in wall following path, immediately stop and return failure
			if (!pathGenerator.checkPointAhead(odometer.getTheta(), 17)){
				rotateSensorsRight(40, false);
				rightAvoidFail = true;
				return false;
			}

			//cap error so speed is not too aggressive
			if(error > 100){
				error = 50;
			}else if (error < -100){
				error = -50;
			}

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

			//if it reaches initial angle + 20 degrees, assumed avoided object successfully and break diriving loop
			if (Math.abs(odometer.getTheta() - endAngle) < ANGLE_THRESH){
				Sound.buzz();
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				break;
			}

		}

		//re-orient sensors
		rotateSensorsRight(40, false);

		//drive forward to get sufficiently past block
		try {Thread.sleep(2500);} catch (InterruptedException e) {}
		return true;
	}


	/**
	 * Filter for ultrasonic readings, for rotating sensor
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


	public void calculateDepositPoint(){
		if(role == 1){
			//tower builder
			if ((gx1 - gx0) == 30.0){
				//x difference is minimum, want to face either +y or -y for best chance at 
				//depositing the block without missing the zone. x deposit coords are constant
				xDeposit = (gx1 + gx0)/2.0;
				xDeposit2 = xDeposit;
				xDeposit3 = xDeposit;
				if(gy0 >= (wy1/2.0)){
					//y deposit is "most-central" of the two y boundaries
					yDeposit = gy0;
					
					//need to reverse further for tower building 
					yDeposit2 = yDeposit - 5.0;
					yDeposit3 = yDeposit - 6.0;
					
					//turn to look "length-wise" down the green zone
					depositAngle = 90.0;
				}else{
					//y deposit is "most-central" of the two y boundaries
					yDeposit = gy1;
	
					//need to reverse further for tower building 
					yDeposit2 = yDeposit + 5.0;
					yDeposit3 = yDeposit + 6.0;
					
					//turn to look "length-wise" down the green zone
					depositAngle = 270.0;
				}
			}else{
				//y difference is minimum, want to face either +x or -x for best chance at 
				//depositing the block without missing the zone. y deposit coords are constant
				yDeposit = (gy1 + gy0)/2.0;
				yDeposit2 = yDeposit;
				yDeposit3 = yDeposit;

				if(gx0 >= (wx1/2.0)){
					//x deposit is "most-central" of the two x boundaries
					xDeposit = gx0;
					
					//need to reverse further for tower building 
					xDeposit2 = xDeposit - 5.0;
					xDeposit3 = xDeposit - 6.0;
					
					//turn to look "length-wise" down the green zone
					depositAngle = 0.0;
				}else{
					xDeposit = gx1;
					
					//need to reverse further for tower building 
					xDeposit2 = xDeposit + 5.0;
					xDeposit3 = xDeposit + 6.0;
					
					//turn to look "length-wise" down the green zone
					depositAngle = 180.0;
				}
			} 
		}else{
			//garbage collector doesn't build towers - each deposit point is the same location
			if ((gx1 - gx0) == 60.0){
				depositAngle = 90.0;
			}else{
				depositAngle = 0.0;
			}
			xDeposit = (gx1 + gx0)/2.0;
			yDeposit = (gy1 + gy0)/2.0;
		}
	}

	/**
	 * Set's the coordinates of the green and red zone. For garbage collector, the zones are passed
	 * in opposite order so it treats the red zone as it's green zone.
	 * @param green Holds the bottom left and top right coordinates of green zone
	 * @param red	Holds the bottom left and top right coordinates of red zone
	 * @param role	The role of tower builder(1) or garbage collector(2)
	 * @param x		Starting x coordinate
	 * @param y		Starting y coordinate
	 */
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




}
