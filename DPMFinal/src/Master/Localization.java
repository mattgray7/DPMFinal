package Master;

import lejos.nxt.*;

public class Localization {
	private static final int SPIN_SPEED = 30;
	private static final int WALL_DISTANCE = 55;
	private static final double CS_DISTANCE = 10.5;

	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	
	private Odometer odo;
	private UltrasonicSensor us;
	private ColorSensor cs;
	private Navigation nav;
	
	public Localization(Odometer odo, UltrasonicSensor us, Navigation nav, ColorSensor cs) {
		this.odo = odo;
		this.cs = cs;
		this.nav = nav;
		this.us = us;
		
		us.off();
	}
	
	public void doLocalization(){
		if (!wallInSight()){
			LCD.drawString("FALLING", 0, 0);
			doLocalizationFallingEdge();
		}
		else{
			LCD.drawString("RISING", 0, 0);
			doLocalizationRisingEdge();
		}
		
		LCD.drawString("New " + odo.getTheta(), 0, 7);
	}
	
	private void doLocalizationFallingEdge(){
		// Rotate left until you don't see a wall
		// This should already be established, and is not really necessary.
		LCD.drawString("Init", 0, 3);
		while(wallInSight()){
			spinLeft();
		}
		
		// Rotate left until you see wall
		LCD.drawString("See wall...", 0, 3);
		while(!wallInSight()){
			spinLeft();
		}
		LCD.drawString("Angle A", 0, 3);
		double angleA = odo.getTheta();
		
		// Rotate right a little to get out of wall
		// Using wallInSight() does not work in all cases, so I spin for a
		// hard-coded 0.5 sec.
		long startTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - startTime < 500){
			spinRight();
		}
		
		// Rotate right until you see a wall
		LCD.drawString("See wall...", 0, 3);
		while(!wallInSight()){
			spinRight();
		}
		LCD.drawString("Angle B", 0, 3);
		double angleB = odo.getTheta();

		// Stop motors
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		
		// Calibrate robot's angle
		LCD.drawString("Calibrate", 0, 3);
		double angleDelta = getAngleCorrection(angleA, angleB);
		odo.setTheta(odo.getTheta() + angleDelta);
	}

	private void doLocalizationRisingEdge(){
		// Rotate left until you see a wall
		// This should already be established, and is not really necessary.
		LCD.drawString("Init", 0, 3);
		while(!wallInSight()){
			spinLeft();
		}
		
		// Rotate until you don't see a wall
		LCD.drawString("Wall...", 0, 3);
		while(wallInSight()){
			spinLeft();
		}
		double angleA = odo.getTheta();
		
		// Rotate right a little to get back in front of a wall
		// Using wallInSight() does not work in all cases, so I spin for a
		// hard-coded 0.5 sec.
		long startTime = System.currentTimeMillis();
		while(System.currentTimeMillis() - startTime < 500){
			spinRight();
		}
		
		// Rotate right until you don't see a wall
		LCD.drawString("Wall...", 0, 3);
		while(wallInSight()){
			spinRight();
		}
		LCD.drawString("Angle B", 0, 3);
		double angleB = odo.getTheta();
		
		// Stop motors
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		
		// Calibrate robot's angle
		LCD.drawString("Calibrate", 0, 3);
		double angleDelta = getAngleCorrection(angleA, angleB);
		odo.setTheta(odo.getTheta() + angleDelta);
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
				if (cs.getNormalizedLightValue() < 485) {
					/*if (i == 0){
						odo.setTheta(88.0);
					}*/
					angles[i] = odo.getTheta();	
					Sound.beep();
					//Sleep thread to detect line only one time
					try {
						Thread.sleep(180);
					}
					catch (Exception e) {}
					spinning = false;
				}
			}
		}
		
		// Calculate angle difference when detecting each line on x and y axis
		thetaY = Math.abs(angles[0] - angles[2]);
		thetaX = Math.abs(angles[1] - angles[3]);
		
		// Calculate coordinates
		x = -CS_DISTANCE * Math.cos(Math.toRadians(thetaY / 2.0));
		y = -CS_DISTANCE * Math.cos(Math.toRadians(thetaX / 2.0));
		
		// Update coordinates in odometer, theta is not updated
		odo.setX(x);
		odo.setY(y);
		
		// Travel to proper (0,0) coordinates
		nav.travelTo(0, 0, false);
		nav.turnTo(90.0, true, true);
		
		try {Thread.sleep(200);} catch (InterruptedException e) {}
		
		// WHAT: why is this called twice?
		// When is is the odometer's theta corrected?
		odo.setX(x);
		odo.setY(y);
		
		cs.setFloodlight(false);
	}
	
	/**
	 * Calculate the angle needed to correct the odometer's heading.
	 * 
	 * This angle just needs to be added to the odometer' angle.
	 * 
	 * @param angleA Angle at which one wall was detected (in rads)
	 * @param angleB Angle at which the other wall was detected (in rads).
	 * 
	 * @return The angle needed to correct the odometer's heading.
	 */
	private double getAngleCorrection(double angleA, double angleB){
		// Method written by Nick. I'd need to draw a picture to explain...
		
		double correction = 0.0;
		double absDiff = Math.abs(angleA - angleB);
		double average = (angleA + angleB) / 2.0;
		
		if(absDiff > 180.0){
			correction = 225.0 - average;
		}
		else{
			correction = 45.0 - average;
		}
		
		LCD.drawString("A " + angleA, 0, 4);
		LCD.drawString("B " + angleB, 0, 5);
		LCD.drawString("C " + correction, 0, 6);
		
		return correction;
	}
	
	/**
	 * Returns true if an object is close enough to be considered a wall.
	 * 
	 * @return True is there is a wall.
	 */
	public boolean wallInSight(){
		if (getFilteredData() <= WALL_DISTANCE){
			return true;
		}else{
			return false;
		}
	}
	
	/**
	 * Get the filtered value for the US's distance.
	 * 
	 * This performs a simple filter, where distances over 100 are snapped
	 * to 200.
	 * 
	 * @return The distance (in cm).
	 */
	private int getFilteredData() {
		int distance;
		
		// This is done in another thread. You need to wait for the ping to
		// complete. This takes > 20 ms.
		us.ping();
		
		// Wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		distance = us.getDistance();
		
		// Snap large distances to 200
		if (distance > 100){
			distance = 200;
		}
				
		return distance;
	}
	
	/**
	 * Spin the robot counter-clockwise.
	 * 
	 * This will not stop on its own.
	 */
	public void spinLeft(){
		leftMotor.setSpeed(SPIN_SPEED); 
		rightMotor.setSpeed(SPIN_SPEED);
		leftMotor.backward();
		rightMotor.forward();
	}
	
	/**
	 * Spin the robot clockwise.
	 * 
	 * This will not stop on its own.
	 */
	public void spinRight(){
		leftMotor.setSpeed(SPIN_SPEED); 
		rightMotor.setSpeed(SPIN_SPEED);
		leftMotor.forward();
		rightMotor.backward();
	}
	
}

