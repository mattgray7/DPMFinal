package Master;

import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.ColorSensor.Color;

public class Navigation extends Thread {

	private NXTRegulatedMotor sensMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;

	private UltrasonicSensor bottomUs;
	private ColorSensor colorSens;
	private ObjectRecognition recog;
	
	private final double POINT_THRESH = 0.5;
	private final double ANGLE_THRESH = 5.0;
	private final double CORNER_ANGLE_THRESH = .5;
	private final int COLOR_THRESH = 330;
	
	private final int FAST = 200;
	private final int JOG = 150;
	private final int SLOW = 100;
	
	private double LW_RADIUS;
	private double RW_RADIUS;
	private double WHEEL_BASE;
	
	private Odometer odometer;
	public Boolean isBusy = true;
	public Boolean isTraveling = false;
	public Boolean isFinished = false;
	public Boolean hasBlock = false;
	public Boolean resetPath = false;
	
	private double theta;
	private double gx0 = 90.0;
	private double gx1 = 120.0;
	private double gy0 = 90.0;
	private double gy1 = 120.0;
	
	private double[] xPath = new double[40];
	private double[] yPath = new double[40];
	
	private double xDestination = 0.0;
	private double yDestination = 0.0;
	private int pathIndex = 0;

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
	public Navigation(Odometer odom, BTSend sender, UltrasonicSensor bot, ColorSensor cs, ObjectRecognition or) {
		bottomUs = bot;
		odometer = odom;
		bts = sender;
		colorSens = cs;
		recog = or;
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
		
		/*while(true){
			Button.waitForAnyPress();
			colorSens.setFloodlight(true);
			recog.checkColor();
		}*/

		//scan();	
		travelTo(30.0, 30.0, false);
		turnTo(90.0, true, true);
		generatePath();
		scan();
		
		//for loop starts at 1 since first elements are current position
		for(int i=1; i < xPath.length; i++){
			if(!hasBlock){
				travelTo(xPath[i], yPath[i], false);
				pathIndex = i;

				if((Math.abs(odometer.getX() - xDestination) < POINT_THRESH + 10) 
						&& (Math.abs(odometer.getY() - yDestination) < POINT_THRESH + 10)){
					Sound.buzz();
					break;
				}
			
				if(resetPath){
					i=1;
					resetPath = false;
				}
			
				scan();
			}else{
				break;
			}
		}
		/*travelTo(0.0, 60.0, false);
		//turnTo(0.0, true, true);
		travelTo(60.0, 60.0, false);
		travelTo(60.0, 0.0, false);
		travelTo(0.0, 0.0, false);*/
		
		
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
		 isBusy = false;
		 double minAng;
		 boolean temp = true; 
		 double distance = Math.sqrt(Math.pow(Math.abs(x -odometer.getX()), 2) + Math.pow(Math.abs(y - odometer.getY()), 2));
		 double masterDist = distance; 
		 double masterAng = 0;
		 colorSens.setFloodlight(Color.RED);
		 while (Math.abs(x - odometer.getX()) > POINT_THRESH || Math.abs(y - odometer.getY()) > POINT_THRESH) {
	  
			 minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0/ Math.PI); 
			 distance = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()), 2));
	  
			LCD.drawString("nx: " + xPath[pathIndex], 0, 5, false);
			LCD.drawString("ny: " + yPath[pathIndex], 0, 6, false);
			 
			 if(colorSens.getNormalizedLightValue() > COLOR_THRESH - 10){
				 leftMotor.stop();
				 rightMotor.stop();
				 isBusy = true;
				 if(recog.checkColor()){
					 capture(18.0);		//special distance that the robot should reverse to lower arm
				 }else{
					 avoid(1);
				 }
				 return;
			 }
			 
			 if (minAng < 0) minAng += 360.0;
	  
			 //Only correct angle during first iteration of the call, otherwise the robot is too oscillatory. 
			 //This is why coordinates are split into 30cmsegments, update every 30cm 
			 if (temp){ 
				 this.turnTo(minAng, true, true);
				 masterAng = minAng;
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
		 isBusy = false;
		 turnTo(masterAng, true, true);
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
		isBusy = true;
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
		isBusy = false; 
	}
	 



	/**
	 * Will stop and scan 180 degrees with both sensors to check for blocks
	 * 
	 * @return void
	 */
	public void scan() {
		double endAngle = odometer.getTheta() - 90.0;	//set end angle of scan
		turnTo(odometer.getTheta() + 90.0, true, true);	//turn to start angle of scan
		
		try {Thread.sleep(500);} catch (InterruptedException e) {}

		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.backward();

		int dist;	//distance read from us sensor
		Boolean object = false;		//true when scan is passing over an object, false in between
		Boolean anyObjectFound = false;		//true if at least one object has been found
		int objDist = 0;	//previously read distanc of object
		
		// assuming 5 is the max objects the robot can distinguish in one square
		int[][] objects = new int[4][20];		//20 is arbitrary size, shouldn't be more than 20 edges per scan
		int objIndex = 0;

		while (odometer.getTheta() > endAngle) {

			dist = getFilteredDistance();		//update the distance

			LCD.drawString("nx: " + xPath[pathIndex], 0, 5, false);
			LCD.drawString("ny: " + yPath[pathIndex], 0, 6, false);

			if (dist < 32) {
				if (object == false) {
					// first reading of object
					object = true;
					objDist = dist;
					objects[0][objIndex] = dist;
					objects[1][objIndex] = (int) odometer.getTheta();
					anyObjectFound = true;
					Sound.beep();
				} else {
					// still reading an object
					objDist = dist;
				}
			}

			if ((Math.abs(objDist - dist) > 9) && (object == true)) {
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
		double ANGLE_OFFSET;
		
		
		//try {bts.sendSignal(1);} catch (IOException e) {}	//1 for lower arms
		

		for (int i = 0; i <= objIndex; i++) {
			double avgDist = (objects[0][i] + objects[2][i]) / 2.0;
			travelDist = (int)avgDist- 10; //-10 for distance between sensor and wheel base
			
			//test offset
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
			
			travelAng = ((objects[1][i] + objects[3][i]) / 2.0 + ANGLE_OFFSET); 
			if (travelDist > 0) {
				if (!hasBlock){
					colorSens.setFloodlight(Color.RED);
					inspect(travelAng, travelDist);
				}else{
					
				}

			}
		}
		colorSens.setFloodlight(false);


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
		double x = odometer.getX();
		double y = odometer.getY();
		double distTraveled = 0;
		turnTo(angle, true, true);
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.forward();
		rightMotor.forward();
		
		while(colorSens.getNormalizedLightValue() < COLOR_THRESH){
			//check for driving too far
			LCD.drawString("cs:" + colorSens.getNormalizedLightValue(), 0, 5);
			

			distTraveled = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()),2));
			
			if(Math.abs(distance - distTraveled) < 0.5){
				break;
			}
			
		}
		
		distTraveled = Math.sqrt(Math.pow(Math.abs(x - odometer.getX()),2) + Math.pow(Math.abs(y - odometer.getY()),2));
		
		leftMotor.stop();
		rightMotor.stop();
		
		//colorSens.setFloodlight();
		try {Thread.sleep(100);} catch (InterruptedException e) {}
		
		//if(colorSens.getNormalizedLightValue() > 330){
		if(recog.checkColor()){
			Sound.beep();
			hasBlock = true;
			capture(distTraveled);		//takes the starting x and y as input
		}else{
			leftMotor.rotate(-convertDistance(LW_RADIUS, distTraveled), true);
			rightMotor.rotate(-convertDistance(RW_RADIUS, distTraveled), false);
			leftMotor.stop();
			rightMotor.stop();
		}

	}
	
	public void capture(double distance){
		
		if(distance < 18){
			distance = 18;
		}
		
		leftMotor.rotate(-convertDistance(LW_RADIUS, distance), true);//back up far enough to bring claw down
		rightMotor.rotate(-convertDistance(RW_RADIUS, distance), false);
		leftMotor.stop();
		rightMotor.stop();
		
		sensMotor.setSpeed(150);
		sensMotor.forward();
		sensMotor.rotate(80, false);
		sensMotor.stop();
		
		try {
			bts.sendSignal(1);
		} catch (IOException e) {
			Sound.buzz();
		}	//1 for lower arms
		
		try {Thread.sleep(2500);} catch (InterruptedException e1) {}
		
		leftMotor.rotate(convertDistance(LW_RADIUS, distance), true);//travel same distance forward with claw down
		rightMotor.rotate(convertDistance(RW_RADIUS, distance), false);//dist may need to be less since the claw is down
		
		leftMotor.setSpeed(JOG + 50);
		rightMotor.setSpeed(JOG - 50);
		leftMotor.forward();
		rightMotor.forward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		leftMotor.setSpeed(JOG - 50);
		rightMotor.setSpeed(JOG + 50);
		leftMotor.forward();
		rightMotor.forward();
		try {Thread.sleep(700);} catch (InterruptedException e1) {}
		leftMotor.setSpeed(JOG);
		rightMotor.setSpeed(JOG);
		leftMotor.rotate(convertDistance(LW_RADIUS, 5), true);
		rightMotor.rotate(convertDistance(RW_RADIUS, 5), false);
		
		
		leftMotor.stop();
		rightMotor.stop();
		
		
		try {bts.sendSignal(2);} catch (IOException e) {Sound.buzz();}	//2 for clamp and raise arms
		
		try {Thread.sleep(1700);} catch (InterruptedException e1) {}
		
		finishLine();
		
	}
	
	/**
	 * Will take the robot from it's current position directly to the bottom left corner of the green zone
	 */
	public void finishLine(){
		//move back into position
		centerSensors();
		
		travelTo(gx0+2, gy0-1, true);
		turnTo(35, true, true);
		
		//move away from claw
		rotateSensorsRight();
		
		try {bts.sendSignal(1);} catch (IOException e) {}	//1 for lower arms and release
		try {Thread.sleep(2500);} catch (InterruptedException e1) {}
		
		
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.rotate(-convertDistance(LW_RADIUS, 10.0), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 10.0), false);
		leftMotor.stop();
		rightMotor.stop();

		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e1) {}
		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e1) {}
		Sound.beep();
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
	    
	    double[] xpath = new double[40];
	    double[] ypath = new double[40];
	    double length = xpath.length;
	    
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
	 * Blocking method that runs to completion and terminates once the robot is
	 * around the block
	 * 
	 * @return void
	 */
	public void avoid(double i) {
		
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.setSpeed(SLOW);
		rightMotor.setSpeed(SLOW);
		leftMotor.rotate(-convertDistance(LW_RADIUS, 8.0), true);
		rightMotor.rotate(-convertDistance(RW_RADIUS, 8.0), false);
		leftMotor.stop();
		rightMotor.stop();
		
		double initAngle = odometer.getTheta();
		turnTo(odometer.getTheta() + 90.0, true, true);
		int dist = getFilteredDistance();
		int BAND_CENTER = 20;
		int error = 0;
		
		
		if (dist > 30){
			//left wall follow
			
			//rotate sensor right
			sensMotor.setSpeed(150);
			sensMotor.forward();
			sensMotor.rotate(40, false);
			sensMotor.stop();
			
			leftMotor.setSpeed(FAST);
			rightMotor.setSpeed(FAST);
			leftMotor.forward();
			rightMotor.forward();
			
			if(i != 1){
				turnTo(initAngle + 45, true, true);
				//drive until a distance is set
				while(dist > BAND_CENTER){
					dist = getFilteredDistance();
					leftMotor.setSpeed(FAST);
					rightMotor.setSpeed(FAST);
					leftMotor.forward();
					rightMotor.forward();
				}
			}
			

			
			while(true){
				dist = getFilteredDistance();
				error = BAND_CENTER - dist;
				
				if(error > 100){
					error = 50;
				}else if (error < -100){
					error = -50;
				}
				//if robot gets to starting x orientation, it has successfully avoided the block
				
				if(error < -2){
					leftMotor.setSpeed(FAST + 50);
					rightMotor.setSpeed(FAST + 30 + (error*2));	//error is negative, right move slower
				}else if (error > 2){
					leftMotor.setSpeed(FAST + 30);
					rightMotor.setSpeed(FAST + 50 + (error * 2));
				}else{
					leftMotor.setSpeed(FAST + 50);
					rightMotor.setSpeed(FAST + 50);
				}
				
				if (Math.abs(odometer.getTheta() - (initAngle - 20)) < ANGLE_THRESH){
					Sound.buzz();
					leftMotor.setSpeed(JOG);
					rightMotor.setSpeed(JOG);
					break;
				}
			}
			
			sensMotor.setSpeed(150);
			sensMotor.backward();
			sensMotor.rotate(-40, false);
			sensMotor.stop();
			
			try {Thread.sleep(1500);} catch (InterruptedException e) {}
	
			
			
		}else{
			
			turnTo(odometer.getTheta() - 180.0, true, true);
			dist = getFilteredDistance();
			if(dist > 30){
				Sound.beep();
				//rotate sensor right
				sensMotor.setSpeed(150);
				sensMotor.backward();
				sensMotor.rotate(-40, false);
				sensMotor.stop();
				
				leftMotor.setSpeed(FAST);
				rightMotor.setSpeed(FAST);
				leftMotor.forward();
				rightMotor.forward();
				
				if(i != 1){
					turnTo(initAngle - 45, true, true);
					//drive until a distance is set
					while(dist > BAND_CENTER){
						dist = getFilteredDistance();
						leftMotor.setSpeed(FAST);
						rightMotor.setSpeed(FAST);
						leftMotor.forward();
						rightMotor.forward();
					}
				}
				
				while(true){
					dist = getFilteredDistance();
					error = BAND_CENTER - dist;
					
					if(error > 100){
						error = 50;
					}else if (error < -100){
						error = -50;
					}
					//if robot gets to starting x orientation, it has successfully avoided the block
					
					if(error < -2){
						leftMotor.setSpeed(FAST + 30 + (error*2));
						rightMotor.setSpeed(FAST + 50);	//error is negative, right move slower
					}else if (error > 2){
						leftMotor.setSpeed(FAST + 30 + (error*2));
						rightMotor.setSpeed(FAST + 50);
					}else{
						leftMotor.setSpeed(FAST + 50);
						rightMotor.setSpeed(FAST + 50);
					}
					
					if (Math.abs(odometer.getTheta() - (initAngle + 20)) < ANGLE_THRESH){
						Sound.buzz();
						leftMotor.setSpeed(JOG);
						rightMotor.setSpeed(JOG);
						break;
					}
					
				}
				//rotate sensor back forward
				sensMotor.setSpeed(150);
				sensMotor.forward();
				sensMotor.rotate(40, false);
				sensMotor.stop();
				
				try {Thread.sleep(1500);} catch (InterruptedException e) {}
				
				
				
			}else{
				//reverse and try again
				turnTo(odometer.getTheta() + 90.0, true, true);
				leftMotor.setSpeed(FAST);
				rightMotor.setSpeed(FAST);
				leftMotor.rotate(-convertDistance(LW_RADIUS, 30.0), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 30.0), false);
				avoid(i+1);
			}
		}
		
		
		//turn to the left
		/*isBusy = false;
		turnTo(odometer.getTheta() + 90.0, true, true);
		int dist = getFilteredDistance();
		
		if (i != 1){
			i *= 0.75;
		}
		
		if(dist > 30){
			 Sound.beep();
			 leftMotor.setSpeed(JOG);
			 rightMotor.setSpeed(JOG);
			 leftMotor.rotate(convertDistance(LW_RADIUS, 30.0*i), true);
			 rightMotor.rotate(convertDistance(RW_RADIUS, 30.0*i), false);
			 Sound.beep();
			 
			 turnTo(odometer.getTheta() - 90.0, true, true);
			 leftMotor.setSpeed(JOG);
			 rightMotor.setSpeed(JOG);
			 leftMotor.rotate(convertDistance(LW_RADIUS, 50.0*i), true);
			 rightMotor.rotate(convertDistance(RW_RADIUS, 50.0*i), false);
			 
			 turnTo(odometer.getTheta() - 90.0, true, true);
			 leftMotor.setSpeed(JOG);
			 rightMotor.setSpeed(JOG);
			 leftMotor.rotate(convertDistance(LW_RADIUS, 30.0*i), true);
			 rightMotor.rotate(convertDistance(RW_RADIUS, 30.0*i), false);
		}else{
			turnTo(odometer.getTheta() - 180.0, true, true);
			if(dist > 30){
				Sound.beep();
				 leftMotor.setSpeed(JOG);
				 rightMotor.setSpeed(JOG);
				 leftMotor.rotate(convertDistance(LW_RADIUS, 30.0*i), true);
				 rightMotor.rotate(convertDistance(RW_RADIUS, 30.0*i), false);
				 
				 leftMotor.setSpeed(JOG);
				 rightMotor.setSpeed(JOG);
				 turnTo(odometer.getTheta() + 90.0, true, true);
				 leftMotor.rotate(convertDistance(LW_RADIUS, 50.0*i), true);
				 rightMotor.rotate(convertDistance(RW_RADIUS, 50.0*i), false);
				 
				 leftMotor.setSpeed(JOG);
				 rightMotor.setSpeed(JOG);
				 turnTo(odometer.getTheta() + 90.0, true, true);
				 leftMotor.rotate(convertDistance(LW_RADIUS, 30.0*i), true);
				 rightMotor.rotate(convertDistance(RW_RADIUS, 30.0*i), false); 
			}else{
				Sound.buzz();
				turnTo(odometer.getTheta() + 90.0, true, true);
				leftMotor.setSpeed(JOG);
				rightMotor.setSpeed(JOG);
				leftMotor.rotate(-convertDistance(LW_RADIUS, 30.0), true);
				rightMotor.rotate(-convertDistance(RW_RADIUS, 30.0), false);
				avoid(i+1);
				return;
			}
		}*/
		
		if(!hasBlock){
			generatePath();		//generate new list of points based on current location
			resetPath = true;	//update the path
		}else{
			finishLine();
		}
		isBusy = true;

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
	
	public void rotateSensorsRight(){
		sensMotor.setSpeed(150);
		sensMotor.forward();
		sensMotor.rotate(80, false);
		sensMotor.stop();
	}
	
	public void centerSensors(){
		sensMotor.setSpeed(150);
		sensMotor.backward();
		sensMotor.rotate(-80, false);
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

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	

}
