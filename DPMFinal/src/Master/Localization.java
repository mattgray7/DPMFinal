package Master;

import lejos.nxt.*;

public class Localization {

	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static int SPIN_SPEED = 80;
	private static int WALL_DISTANCE = 55;
	
	private NXTRegulatedMotor leftMotor = Motor.B, rightMotor = Motor.C;
	private Odometer odo;
	private UltrasonicSensor us;
	private LocalizationType locType;
	
	public Localization(Odometer odo, UltrasonicSensor us) {
		this.odo = odo;
		this.us = us;
		us.off();
	}
	
	public void doLocalization(){
		if (!wallInSight()){
			LCD.drawString("FALLING", 0, 0);
			locType = LocalizationType.FALLING_EDGE;
			doLocalizationFallingEdge();
		}
		else{
			LCD.drawString("RISING", 0, 0);
			locType = LocalizationType.RISING_EDGE;
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
		// Tutorial equations, swapped because odometer orientation is swapped
		
		double correction = 0.0;
		double absDiff = Math.abs(angleA - angleB);
		double average = (angleA + angleB) / 2.0;
		
		if(absDiff > 180.0){
			correction = 225.0 - average;
		}
		else{
			correction = 45.0 - average;
		}
		
		/*
		if (angleA > angleB) {
			Sound.buzz();
			correction= 230.0 - ((angleA + angleB) / 2.0);	// Tweaked for looking at wall
		}
		else{
			Sound.beep();
			correction = 45.0 - ((angleA + angleB) / 2.0);
		}
		*/
		
		LCD.drawString("A " + angleA, 0, 4);
		LCD.drawString("B " + angleB, 0, 5);
		LCD.drawString("C " + correction, 0, 6);
		
		return correction;
	}
		
	//returns true if a wall is close enough to be considered
	public boolean wallInSight(){
		if (getFilteredData() <= WALL_DISTANCE){
			return true;
		}else{
			return false;
		}
	}
	
	private int getFilteredData() {
		int distance;
		
		// The US can operate in two modes: ping and continuous.
		// In ping mode, you need to call ping() and wait > 20 ms, then call
		// getDistance(). Calling ping() disables continuous mode.
		// In continuous mode, the US automatically pings distance all the time.
		// You can call getDistance at any time, which will give the latest
		// distance value.
		
		// This is done in parrallel. You need to wait for the ping to
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
	
	public void spinLeft(){
		leftMotor.setSpeed(SPIN_SPEED); 
		rightMotor.setSpeed(SPIN_SPEED);
		leftMotor.backward();	//turn left
		rightMotor.forward();
	}
	
	public void spinRight(){
		leftMotor.setSpeed(SPIN_SPEED); 
		rightMotor.setSpeed(SPIN_SPEED);
		leftMotor.forward();
		rightMotor.backward();	//turn right
	}
	
}
