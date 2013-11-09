package Slave;

import java.util.ArrayList;
import java.util.List;

import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.robotics.Color;

public class ObjectRecognition {
	private final double COS_THETA_MARGIN = 0.001;
	private final double LENGTH_THRESHOLD = 20.0;
	
	private ColorSensor colorSensor;
	private Vector calibratedBlueColor;

	/**
	 * Constructor
	 * @param cs The color sensor located on the claw
	 */
	public ObjectRecognition(ColorSensor cs){
		colorSensor = cs;
		calibratedBlueColor = new Vector(0.0, 0.0, 0.0);
	}
	
	
	/**
	 * Check the distance from the sensor to the object, may lead to signal being sent
	 * back to master to reposition robot in front of the block
	 * @return The distance from the sensor to the object
	 */
	public int checkDistance(){
		return 0;
	}
	
	
	/**
	 * Checks the color of the object
	 * @return True if the object is a blue block, false otherwise
	 */
	public Boolean checkColor(){
		Color c = colorSensor.getColor();
		
		Vector currentColor = new Vector(c.getRed(), c.getGreen(), c.getBlue());
		double cosTheta = Vector.dot(calibratedBlueColor,  currentColor);
		
		boolean isBlueBlock = (Math.abs(1.0 - cosTheta) < COS_THETA_MARGIN) &&
				              currentColor.length() > LENGTH_THRESHOLD;
		
		return isBlueBlock;
	}
	
	/**
	 * Prompts the user to perform a calibration of the blue block's color.
	 * This will obtain multiple samples of the blue color and retain the
	 * average.
	 */
	public void calibrateBlueBlock(){
		printCalibrationMessage();
		
		// Need to provide an ArrayList since we don't know how many calibrations
		// the user wants to do
		List<Vector> colors = new ArrayList<Vector>();
		
		int calibrationNumber = 1;
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE){
			LCD.clear(5);
			LCD.drawString("#" + calibrationNumber, 0, 5);
			
			Color c = colorSensor.getColor();
			Vector color = new Vector(c.getRed(), c.getGreen(), c.getBlue());
			
			colors.add(color);
			
			calibrationNumber++;
		}
		
		// This takes the ArrayList, transforms it to an array, and gets
		// the average color
		calibratedBlueColor = Vector.average((Vector[])colors.toArray());
	}
	
	/**
	 * Prints a message about the calibration.
	 * 
	 * Explains which buttons to press.
	 */
	private void printCalibrationMessage(){
		LCD.clear();
		LCD.drawString("Calibration:", 0, 0);
		LCD.drawString("Press to calibr.", 0, 2);
		LCD.drawString("Escape to stop", 0, 3);
	}
}
