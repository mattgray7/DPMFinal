package Master;

import lejos.nxt.*;

public class Localization {

	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	private static double ROTATION_SPEED = 30;
	private final double cSensorDist = 10.5;
	private NXTRegulatedMotor leftMotor = Motor.B, rightMotor = Motor.C;
	private Odometer odo;
	private UltrasonicSensor us;
	private LocalizationType locType;
	private ColorSensor cs;
	private Navigation nav;
	//private Navigation nav;
	
	public Localization(Odometer odo, UltrasonicSensor us, Navigation navi, ColorSensor colsens) {
		this.odo = odo;
		//this.nav = nav;
		this.cs = colsens;
		this.nav = navi;
		this.us = us;
		this.locType = locType;
		us.off();
	}
	
	public void doLocalization() {
		double angleA, angleB;
		double theta = 0.0;
		
		//int buttonChoice = Button.waitForAnyPress();
		
		//determine localization type
		if (wallInSight()) { 
			locType = LocalizationType.FALLING_EDGE;
		} else {
			locType = LocalizationType.RISING_EDGE;
		}
		
		if (locType == LocalizationType.FALLING_EDGE) {
			// rotate the robot until it sees no wall
			
			while(wallInSight()){
				spinLeft();
			}
			
			angleA = odo.getTheta();
			
			// keep rotating until the robot sees a wall, then latch the angle
			leftMotor.forward();
			rightMotor.backward();
			try { Thread.sleep(100); } catch (InterruptedException e) {}
			//rotate until the wall is picked up again
			while(!wallInSight()){
				spinRight();
			}
			//keep rotating until the robot sees no wall, latch angle
			while(wallInSight()){
				spinRight();
			}
			
			angleB = odo.getTheta();

			//tutorial equations, swapped because odometer orientation is reversed (y axis = 90 degrees)
			if (angleA > angleB) {
				theta = 230 - ((angleA + angleB)/2);
				theta += odo.getTheta();
			} else if (angleA < angleB) {
				theta = 45.0 - ((angleA + angleB)/2);
				theta += odo.getTheta();
			}

			odo.setX(-15.0);	//reset x and y positions
			odo.setY(-15.0);
			odo.setTheta(theta);	//update new theta
			//nav.travelTo(0.0,0.0,false);
			
			nav.turnTo(90.0,true, true);
			
			
			//nav.turnTo(90.0,true);	//only for test angle measurement
			
			//turn to face back wall, get y distance
		    /*nav.turnTo(270.0, false);	
			double yDist = getFilteredData();
			yDist -= 28; 	//subtract the length of the tile minus the distance from sensor to wheel base
	
			//turn to face left wall, get x distance
			nav.turnTo(180.0, false);
			double xDist = getFilteredData();
			xDist -= 28;	//subtract the length of the tile minus the distance from sensor to wheel base
			
			odo.setX(xDist);
			odo.setY(yDist);
			
			// Travel close to the intersection. If the robot were to go directly 
			// to (0,0), the light sensor would immediately read a line and the
			// line count would be off
			nav.travelTo(-2.0,-2.0);
			nav.turnTo(90.0, true);*/

		} else {
			//this one runs
			//spin until a wall is read, latch the angle
			while(!wallInSight()){
				spinRight();
			}
			angleA = odo.getTheta();
			
			leftMotor.backward();
			rightMotor.forward();
			try { Thread.sleep(100); } catch (InterruptedException e) {}
			
			//rotate left until a wall is read, latch the angle
			while(wallInSight()){
					spinLeft();
			}
			while(!wallInSight()){
				spinLeft();
			}
			
			angleB = odo.getTheta();
			
			//tutorial equations, swapped because odometer orientation is swapped
			if (angleA > angleB) {
				Sound.buzz();
				theta = 240 - ((angleA + angleB)/2);
				theta += odo.getTheta();
			} else if (angleA < angleB) {
				Sound.beep();
				theta = 45.0 - ((angleA + angleB)/2);
				theta += odo.getTheta();
			}
			
			//reset x and y position, update theta
			odo.setX(-10.0);
			odo.setY(-5.0);
			odo.setTheta(theta);
			//nav.travelTo(0.0,0.0,false);
			
			nav.turnTo(90.0,true, true);
			try {Thread.sleep(500);} catch (InterruptedException e) {}

			
			
			//nav.turnTo(90.0,true); //test for angle measurement
			
			//face back wall, get y distance
			/*nav.turnTo(270.0, false);
			double yDist = getFilteredData();
			yDist -= 28;	//offset for tile length and distance from sensor to wheel base
			
			//face left wall, get x distance
			nav.turnTo(180.0, false);
			double xDist = getFilteredData();
			xDist -= 28;	//offset for tile length and distance from sensor to wheel base
			
			//update x and y coordinates
			odo.setX(xDist);
			odo.setY(yDist);
			
			//travel close to (0,0), but not exactly 
			nav.travelTo(-2.0,-2.0);
			nav.turnTo(90.0, true);//90
			
			leftMotor.forward();
			rightMotor.forward();*/

		}
	}
	
	public void doLightLocalization() {
		nav.travelTo(0.0,0.0,false);
		Sound.buzz();
		boolean spinning;
		double thetaX;
		double thetaY;
		double x;
		double y;
		//Angle array
		double[] angles = new double[4];
		//nav.travelTo(8, 8, true);
		cs.setFloodlight(true);
		//Spin and clock angles at which black lines are detected
		spinLeft();
		for (int i=0; i<4; i++) {
			spinning = true;
			while (spinning) {
				LCD.clear();
				LCD.drawInt(cs.getNormalizedLightValue(), 0, 7);
				if (cs.getNormalizedLightValue() < 490) {
					angles[i] = odo.getTheta();	
					Sound.beep();
					//Sleep thread to detect line only one time
					try {
						Thread.sleep(100);
					}
					catch (Exception e) {}
					spinning = false;
				}
			}
		}
		
		//Calculate angle difference when detecting each line on x and y axis
		thetaY = Math.abs(angles[0] - angles[2]);
		thetaX = Math.abs(angles[1] - angles[3]);
		//Calculate coordinates
		x = -cSensorDist * Math.cos(Math.toRadians(thetaY/2));
		y = -cSensorDist * Math.cos(Math.toRadians(thetaX/2));
		//Update coordinates in odometer, theta is not updated
		odo.setX(x);
		odo.setY(y);
		//Travel to proper (0,0) coordinates
		nav.travelTo(0, 0, false);
		nav.turnTo(90.0, true, true);
		odo.setX(x);
		odo.setY(y);
		
	}
	
	//returns true if a wall is close enough to be considered
	public boolean wallInSight(){
		if (getFilteredData() <= 40){
			return true;
		}else{
			return false;
		}
	}
	
	private int getFilteredData() {
		int distance;
		
		// do a ping
		us.ping();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		distance = us.getDistance();
		
		//clip large distances
		if (distance > 100){
			distance = 200;
		}
				
		return distance;
	}
	
	public void spinLeft(){
		leftMotor.setSpeed(200); 
		rightMotor.setSpeed(200);
		leftMotor.backward();	//turn left
		rightMotor.forward();
	}
	
	public void spinRight(){
		leftMotor.setSpeed(200); 
		rightMotor.setSpeed(200);
		leftMotor.forward();
		rightMotor.backward();	//turn right
	}
	
}

