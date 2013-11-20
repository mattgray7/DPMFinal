package Master;

import lejos.nxt.*;

public class Localization {
	private static final int SPIN_SPEED = 100;
	private static final int WALL_DISTANCE = 55;
	private static final double CS_DISTANCE = 10.5;
	private static final int BLACK_LINE_THRESHOLD = 485;

	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	
	private Odometer odo;
	private UltrasonicSensor us;
	private ColorSensor cs;
	private Navigation nav;
	
	/**
	 * Constructor
	 * @param odo The shared Odometer
	 * @param us Ultrasonic sensor used for ultrasonic localization
	 * @param navi	Main navigation class to turn and drive to origin
	 * @param colsens	Color sensor used for light localization
	 */
	public Localization(Odometer odo, UltrasonicSensor us, Navigation navi, ColorSensor colsens) {
		this.odo = odo;
		this.cs = cs;
		this.nav = nav;
		this.us = us;
		
		us.off();
	}

	
	/**
	 * Performs rising or falling edge ultrasonic localization depending on
	 * if the sensor initially reads a wall.
	 */
	public void doLocalization(){
		if (!wallInSight()){
			LCD.drawString("FALLING", 0, 0);
			doUSLocalizationFallingEdge();
		}
		else{
			LCD.drawString("RISING", 0, 0);
			doUSLocalizationRisingEdge();
		}
	}
	
	private void doUSLocalizationFallingEdge(){
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
		
		// Rotate right a little to get out of wall.
		// Using wallInSight() does not work in all cases, so I spin for a
		// hard-coded 0.5 sec.
		long startTime = System.currentTimeMillis();
		long spinTime = 500;
		Sound.beep();
		spinRight();
		while(System.currentTimeMillis() - startTime < spinTime){
			// Wait
		}
		Sound.beep();
		
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
		usLocalizationCorrection(angleA, angleB);
	}

	private void doUSLocalizationRisingEdge(){
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
		long spinTime = 500;
		Sound.beep();
		spinRight();
		while(System.currentTimeMillis() - startTime < spinTime){
			// Wait
		}
		Sound.beep();
		
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
		usLocalizationCorrection(angleA, angleB);
	}
	
	/**
	 * Performs light localization around a grid intersection. Method assumes robot is close to an intersection
	 */
	//Need to adjust angle offset for right light sensor
	public void doLightLocalization() {
		boolean hasSeenLine;
		
		double thetaX;
		double thetaY;
		double x;
		double y;
		double[] angles = new double[4];
		
		cs.setFloodlight(true);
		
		//Spin and clock angles at which black lines are detected
		spinLeft();
		for (int i=0; i<4; i++) {
			hasSeenLine = false;
			
			while (!hasSeenLine) {
				LCD.clear();
				LCD.drawInt(cs.getNormalizedLightValue(), 0, 7);
				
				if (cs.getNormalizedLightValue() < BLACK_LINE_THRESHOLD) {
					angles[i] = odo.getTheta();	
					Sound.beep();
					hasSeenLine = true;
					
					//Sleep thread to detect line only one time
					long startTime = System.currentTimeMillis();
					while(System.currentTimeMillis() - startTime < 250){
						// Wait
					}
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
		
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		
		cs.setFloodlight(false);
	}
	
	/**
	 * Reset the odometer using the two angles from US localization.
	 * 
	 * @param angleA The angle at which the first wall was detected (in degrees).
	 * @param angleB The angle at which the second wall was detected (in degrees).
	 */
	private void usLocalizationCorrection(double angleA, double angleB){
		double angleDelta = getAngleCorrection(angleA, angleB);
		double distance = getDistanceToWall(angleA, angleB);
		
		odo.setTheta(odo.getTheta() + angleDelta);
		odo.setX(distance - 30.0);
		odo.setY(distance - 30.0);
	}
	
	/**
	 * Calculate the angle needed to correct the odometer's heading.
	 * 
	 * This angle just needs to be added to the odometer' angle.
	 * 
	 * @param angleA The angle at which the first wall was detected (in degrees).
	 * @param angleB The angle at which the second wall was detected (in degrees).
	 * 
	 * @return The angle to add to the odometer's heading, such that the heading is corrected.
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
		
		return correction;
	}
	
	/**\
	 * Get an approximation of the robot's initial position after US localization.
	 * 
	 * This uses the angles at which the two walls were detected. It assumes
	 * the x and y distances are the same.
	 * 
	 * @param angleA The angle at which the first wall was detected (in degrees).
	 * @param angleB The angle at which the second wall was detected (in degrees).
	 * 
	 * @return The distance from the robot to the wall.
	 */
	private double getDistanceToWall(double angleA, double angleB){
		double difference = Math.abs(angleA - angleB);
		
		// In case of weird angleA / angleB not in the range [0, 360]. Shouldn't happen.
		difference = Odometer.fixDegAngle(difference);
		
		// We want the smallest angle between A and B
		if(difference > 180.0){
			difference = 360.0 - difference;
		}
		
		// The robot should have recorded that the angle between the
		// two walls is exactly 90.0 degrees. However, since it is a little far
		// away from the wall's corner and there is a threshold for what
		// is considered a wall, it will see a little bit more than 90.0 degrees.
		double excess = difference - 90.0;
		
		if(excess < 0.0){
			return 0.0;
		}
		else{
			double triangleHypothenuse = WALL_DISTANCE;
			double triangleAngle = (excess / 2.0) * Math.PI / 180.0;
			double triangleOppositeSide = triangleHypothenuse * Math.asin(triangleAngle);
			
			// In short, the US sees larger distances (than in real life),
			// so we need to return a smaller distance.
			return triangleOppositeSide / 1.5;	// Tweaked value
		}
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

