package Master;

import lejos.nxt.ColorSensor.*;
import lejos.nxt.*;
//import lejos.nxt.ColorSensor.Color;
import lejos.util.Timer;


public class OdometryCorrection extends Thread{
	Odometer odometer;
	ColorSensor leftSensor = new ColorSensor(SensorPort.S1);
	ColorSensor rightSensor = new ColorSensor(SensorPort.S2);
	int[] leftValues = new int[10];
	int[] rightValues = new int[10];
	private static final int CORRECTION_PERIOD = 10;
	private static final int ANGLE_THRESH = 10; //angle threshold of when to correct angle
	private static final int POINT_THRESH = 10; //distance threshold of which point to snap to
	private static final int DISTANCE_FROM_LSENSOR = 5; //distance from sensor to center
	private final double[] gridLines = {0.0, 30.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0, 360.0, 390.0, 410.0};		//missing the last wall line?
	private Boolean lineChange = false;
	private Boolean leftLineRead = false;
	private Boolean rightLineRead = false;
	private Boolean acceptRead = false;
	
	
	private long t1;
	private long t2;
	private double newTheta = 90.0;
	private final double WHEEL_RADIUS = 2.1;
	private final double SENSOR_DIST = 17.0;
	
	//constructor
	public OdometryCorrection(Odometer odom){
		this.odometer = odom;
	}
	
	
	//start thread
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
	
	public void fillWindows(){
		for(int i=0; i < 10; i++){
			leftValues[i] = leftSensor.getNormalizedLightValue();
			rightValues[i] = rightSensor.getNormalizedLightValue();
		}
	}
	
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
	
	public double average(int[] arr){
		int sum=0;
		for(int i=0; i< arr.length; i++){
			sum += arr[i];
		}
		return (double)(sum/arr.length);
	}
	
	//modifies the xError and yError attributes depending on light value readings
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
	
	
	/*
	 * Modifies the thetaError attribute but ONLY at certain times. Should only update
	 * when the robot is around 0, 90, 180, or 270 degrees and both light sensors read
	 * a line within a certain time threshold. This should (hopefully) prevent random
	 * line crossings from messing with the odometer
	 */
	public void calculateThetaError(){
		if(compareAverageWithCurrent(leftValues, leftSensor)){
			t1 = System.currentTimeMillis();
			if ((t1 - t2) < 50){	//have no idea what this number should be - time interval between lines read
				acceptRead = true;
			}
			
		}
		
		if(compareAverageWithCurrent(rightValues, rightSensor)){
			t2 = System.currentTimeMillis();
			if ((t2 - t1) < 50){	//have no idea what this number should be - time interval between lines read
				acceptRead = true;
			}
		}
		
	}
	
	
}
