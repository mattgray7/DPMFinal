package Master;

import lejos.nxt.ColorSensor.*;
import lejos.nxt.*;
//import lejos.nxt.ColorSensor.Color;
import lejos.util.Timer;


public class OdometryCorrection extends Thread{
	Odometer odometer;
	ColorSensor leftSensor;
	ColorSensor rightSensor;
	int[] leftValues = new int[10];
	int[] rightValues = new int[10];
	private static final int CORRECTION_PERIOD = 10;
	private static final int ANGLE_THRESH = 10; //angle threshold of when to correct angle
	private static final int POINT_THRESH = 10; //distance threshold of which point to snap to
	private static final int DISTANCE_FROM_LSENSOR = 5; //distance from sensor to center
	private static final int TIME_THRESH = 50; //Threshold that line readings are accepted in
	
	private final double[] gridLines = {0.0, 30.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0, 360.0, 390.0, 410.0};		//missing the last wall line?
	private Boolean acceptRead = false;
	
	
	private long t1;
	private long t2;
	private double newTheta = 90.0;
	private final double WHEEL_RADIUS = 2.1;
	private final double SENSOR_DIST = 17.0;
	
	/**
	 * Constructor
	 * @param odom The shared odometer amongst classes
	 * @param l	The left side color sensor
	 * @param r The right side color sensor
	 */
	public OdometryCorrection(Odometer odom, ColorSensor l, ColorSensor r){
		this.odometer = odom;
		leftSensor = l;
		rightSensor = r;
	}
	
	
	/**
	 * Main execution of odometry correction. The odometer will only be corrected when the 
	 * orientation of the robot is around 0, 90, 180, or 270 degrees. This should prevent lines read
	 * during turning from affecting the odometer
	 * @return void
	 */
	public void run(){
		long correctionStart, correctionEnd;
		leftSensor.setFloodlight(Color.RED);
		rightSensor.setFloodlight(Color.RED);

		
		fillWindows();
		
		
		while (true) {
			correctionStart = System.currentTimeMillis();
			
			if( (Math.abs(odometer.getTheta()) < ANGLE_THRESH) 
				|| (Math.abs(odometer.getTheta() - 90.0) < ANGLE_THRESH) 
				|| (Math.abs(odometer.getTheta() - 180.0) < ANGLE_THRESH) 
				|| (Math.abs(odometer.getTheta() - 270.0) < ANGLE_THRESH) ){
				
				calculateThetaError();
			}
				
			if(acceptRead){
				newTheta = Math.atan((SENSOR_DIST*180)/((t2-t1)*(Motor.A.getSpeed() * Math.PI * WHEEL_RADIUS)));
				newTheta = ((newTheta * 180)/Math.PI);
				odometer.setTheta(newTheta);
				acceptRead = false;
				calculatePositionError(); //may need to be before corrected angle depnding on what we do
				try { Thread.sleep(300); }catch (InterruptedException e) {}		//to avoid reading the same line?
			}
			
			
			// this ensure the odometry correction occurs only once every period - IS THIS NECESSARY/A PROBLEM
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
			
		}
	}
	

	
	
	/**
	 * Modifies the thetaError attribute but ONLY at certain times. Line readings will only
	 * be accepted if they are within a certain time threshold.
	 * @return void
	 */
	public void calculateThetaError(){
		if(compareAverageWithCurrent(leftValues, leftSensor)){
			t1 = System.currentTimeMillis();
			if ((t1 - t2) < TIME_THRESH){	//have no idea what this number should be - time interval between lines read
				acceptRead = true;
			}
			
		}
		
		if(compareAverageWithCurrent(rightValues, rightSensor)){
			t2 = System.currentTimeMillis();
			if ((t2 - t1) < TIME_THRESH){	//have no idea what this number should be - time interval between lines read
				acceptRead = true;
			}
		}
		
	}
	
	
	/**
	 * Modified the x and y position based on orientation and proximity to closest grid line
	 * @return void
	 */
	public void calculatePositionError(){
		
		for(int i=0; i<gridLines.length; i++){
			
			if( (Math.abs(odometer.getTheta()) < ANGLE_THRESH) 
				&& (Math.abs((odometer.getX() - DISTANCE_FROM_LSENSOR) - gridLines[i]) < POINT_THRESH)){	//distance from lensor will need to change
					//going "right"
					odometer.setX(gridLines[i] + DISTANCE_FROM_LSENSOR);
					
			}else if ( (Math.abs(odometer.getTheta() - 90.0) < ANGLE_THRESH) 
				&& (Math.abs((odometer.getY() - DISTANCE_FROM_LSENSOR) - gridLines[i]) < POINT_THRESH)){
					//going "up"
					odometer.setY(gridLines[i] + DISTANCE_FROM_LSENSOR);
					
			}else if ( (Math.abs(odometer.getTheta() - 180.0) < ANGLE_THRESH) 
				&& (Math.abs((odometer.getX() + DISTANCE_FROM_LSENSOR) - gridLines[i]) < POINT_THRESH)){
					//going "left"
					odometer.setX(gridLines[i] - DISTANCE_FROM_LSENSOR);
					
			}else if ( (Math.abs(odometer.getTheta() - 270.0) < ANGLE_THRESH)
				&& (Math.abs((odometer.getY() + DISTANCE_FROM_LSENSOR) - gridLines[i]) < POINT_THRESH)){
					//going "down"
					odometer.setY(gridLines[i] - DISTANCE_FROM_LSENSOR);
			}
			
		}
		
	}

	/**
	 * Compares the average of a list of values with a new value
	 * @param array The list of values that the average will be calculated from
	 * @param sens	The sensor whose reading will be compared to the list of values
	 * @return True if the comparison implies that a line has been read
	 */
	public Boolean compareAverageWithCurrent(int[] array, ColorSensor sens){
		
		double avg = average(array);
		
		int s1 = sens.getNormalizedLightValue();
		int s2 = sens.getNormalizedLightValue();
		
		double lightValues = (double)(s1 + s2)/ 2.0;
		
		if (Math.abs(lightValues - avg) > 15){
			return true;
		}
		
		return false;
	}
	
	/**
	 * Fills the base light value arrays with wood-colored readings
	 * @return void
	 */
	public void fillWindows(){
		for(int i=0; i < 10; i++){
			leftValues[i] = leftSensor.getNormalizedLightValue();
			rightValues[i] = rightSensor.getNormalizedLightValue();
		}
	}
	
	/**
	 * Computes the average of an array
	 * @param arr The array to be computed
	 * @return double - The average of the input array
	 */
	public double average(int[] arr){
		int sum=0;
		for(int i=0; i< arr.length; i++){
			sum += arr[i];
		}
		return (double)(sum/arr.length);
	}
	

	
}
