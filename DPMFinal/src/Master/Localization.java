package Master;

import lejos.nxt.*;
import lejos.nxt.comm.RConsole;


/**
 * Class to control the robot's movement.
 * <p>
 * Allows the robot to turn and travel to a destination, while avoiding obstacles.
 * 
 * @author Julien Liberta
 * @author Matt Gray
 */
public class Localization {
	private static final int SPIN_SPEED = 200;
	private static final int WALL_DISTANCE = 55;
	private static final Vector CS_POSITION = new Vector(-8.7, -10.5, 0.0);	// Back-left CS
	private static final int BLACK_LINE_SLOPE_THRESHOLD = -8;	// By looking at graph of filtered values during light localization

	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	
	private Odometer odo;
	private UltrasonicSensor us;
	private ColorSensor cs;
	private Navigation nav;
	
	private MedianFilter usFilter;
	private SmoothDifferenceFilter csFilter;
	
	/**
	 * Constructor
	 * @param odo The shared Odometer
	 * @param us Ultrasonic sensor used for ultrasonic localization
	 * @param nav	Main navigation class to turn and drive to origin
	 * @param cs	Color sensor used for light localization
	 */
	public Localization(Odometer odo, UltrasonicSensor us, Navigation nav, ColorSensor cs) {
		this.odo = odo;
		this.cs = cs;
		this.nav = nav;
		this.us = us;
		
		this.usFilter = new MedianFilter(3);
		this.csFilter = new SmoothDifferenceFilter(6);
		
		us.off();
	}

	
	/**
	 * The intended behavior was to choose between rising and falling edge localization,
	 * depending on whether the robot initially faced a wall or not, but it turns out
	 * falling edge does not work.
	 * <p>
	 * So it always does rising edge.
	 */
	public void doLocalization(){
		/*
		if (!wallInSight()){
			LCD.drawString("FALLING", 0, 0);
			doUSLocalizationFallingEdge();
		}
		else{
			LCD.drawString("RISING", 0, 0);
			doUSLocalizationRisingEdge();
		}
		*/
		
		//doUSLocalizationRisingEdge();
		us.continuous();
		doUSLocalizationRisingEdge();
		us.off();
	}
	
	/**
	 * Perform US localization using falling edge.
	 *
	 */
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
		long spinTime = 1000;
		Sound.beep();
		spinRight();
		while(System.currentTimeMillis() - startTime < spinTime){
			// Wait
			getFilteredDistance();
			//RConsole.println("WHILE dist " + usFilter.getFilteredValue() + "\n");
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

	
	/**
	 * Perform US localization using rising edge.
	 * 
	 */
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
		long spinTime = 1000;
		Sound.beep();
		spinRight();
		while(System.currentTimeMillis() - startTime < spinTime){
			// Wait
			getFilteredDistance();
			//RConsole.println("WHILE dist " + usFilter.getFilteredValue() + "\n");
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
	 * Performs light localization around a grid intersection. This assumes the robot is
	 * close to an intersection.
	 */
	public void doLightLocalization() {
		boolean hasSeenLine;
		
		double thetaX;
		double thetaY;
		double x;
		double y;
		double[] angles = new double[4];
		
		cs.setFloodlight(true);
		csFilter.empty();
		
		// Because the color sensor is to in the back-left, you are always
		// going to record the first y-line first.
		nav.turnTo(90, false, true);
		
		//Spin and clock angles at which black lines are detected
		spinLeft();
		//nav.setSpeeds(0,  0);
		for (int i=0; i<4; i++) {
			hasSeenLine = false;
			
			while (!hasSeenLine) {
				csFilter.add(cs.getNormalizedLightValue());

				LCD.clear();
				LCD.drawString("FCS " + csFilter.getFilteredValue(), 0, 1);
				LCD.drawString("FCS " + csFilter.getRawValue(), 0, 2);
				
				if (csFilter.getFilteredValue() < BLACK_LINE_SLOPE_THRESHOLD) {
					double odoAngle = odo.getTheta();
					// Because getTheta returns the angle from the front of the robot,
					// we substract that part from the CS's angle.
					double csAngle = CS_POSITION.xyAngleDeg() - 90.0;
					double lineAngle = Odometer.fixDegAngle(odoAngle + csAngle);
					
					angles[i] = lineAngle;
					Sound.beep();
					hasSeenLine = true;
					
					//Sleep thread to detect line only one time
					long startTime = System.currentTimeMillis();
					while(System.currentTimeMillis() - startTime < 250){
						// Wait
						csFilter.add(cs.getNormalizedLightValue());
						LCD.drawString("WAIT", 0, 0);
					}
				}
			}
		}

		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		
		// Find angle correction
		double positiveXAxis = MyMath.averageAngle(angles[0], angles[2]);
		double positiveYAxis = MyMath.averageAngle(angles[1], angles[3]);

		double correctionXAxis;
		double correctionYAxis;
		
		if(positiveXAxis < 90.0 || positiveXAxis > 270.0){
			correctionXAxis = MyMath.correctionDeg(positiveXAxis, 0.0);
		}
		else{
			correctionXAxis = MyMath.correctionDeg(positiveXAxis, 180.0);
		}

		if(positiveYAxis > 0.0 && positiveYAxis < 180.0){
			correctionYAxis = MyMath.correctionDeg(positiveYAxis, 90.0);
		}
		else{
			correctionYAxis = MyMath.correctionDeg(positiveYAxis, 270.0);
		}
		
		double averageCorrection = (correctionXAxis + correctionYAxis) / 2.0;

		double odoTheta = odo.getTheta();
		double correctedAngle = MyMath.fixAngleDeg(odoTheta + averageCorrection + (-0.0));	// Adding (-10), tweaked value.

		odo.setTheta(correctedAngle);
	}
	
	/**
	 * Reset the odometer using the two angles from US localization.
	 * 
	 * @param angleA The angle at which the first wall was detected (in degrees).
	 * @param angleB The angle at which the second wall was detected (in degrees).
	 */
	private void usLocalizationCorrection(double angleA, double angleB){
		double angleDelta = getAngleCorrection(angleA, angleB);
		// The following is not used, because a hard-coded distance
		// was more reliable.
		//double distance = getDistanceToWall(angleA, angleB);
		double distance = 22;
		
		odo.setTheta(odo.getTheta() + angleDelta);
		odo.setX(distance - 30.0);
		odo.setY(distance - 30.0);
	}
	
	/**
	 * Calculate the angle needed to correct the odometer's heading.
	 * <p>
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
	 * <p>
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
		
		// In case of weird angleA or angleB not in the range [0, 360]. Shouldn't happen.
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
			return triangleOppositeSide / 1.7;	// Tweaked value
		}
	}
	
	/**
	 * Returns true if an object is close enough to be considered a wall.
	 * 
	 * @return True is there is a wall.
	 */
	public boolean wallInSight(){
		if (getFilteredDistance() <= WALL_DISTANCE){
			return true;
		}else{
			return false;
		}
	}
	
	/**
	 * Get the filtered value for the US's distance.
	 * <p>
	 * This performs a simple filter, where distances over 100 are snapped
	 * to 200.
	 * 
	 * @return The distance (in cm).
	 */
	private int getFilteredDistance() {
		usFilter.add(us.getDistance());
				
		return usFilter.getFilteredValue();
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

