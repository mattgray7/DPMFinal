package Master;

import java.util.ArrayList;
import java.util.List;

import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.robotics.Color;

/**
 * Class to recognize whether a block is a blue block or not.
 * 
 * First, you should calibrate the blue block's color by calling the
 * calibrateBlueBlock() method. If you don't, then it won't consider anything
 * to be a blue block.
 * 
 * Then you can call checkColor() to verify if an object is blue or not.
 * Objects that aren't bright enough will be rejected, so closer blue blocks
 * have more chances or being recognized as blue block.
 * 
 * @author Nick
 *
 */
public class ObjectRecognition {
	private final double COS_THETA_MARGIN = 0.005;
	private final double LENGTH_THRESHOLD = 10.0;
	
	private ColorSensor colorSensor;
	private Vector calibratedBlueColor;
	
	
	/**
	 * Call this function if you want to quickly setup the object
	 * recognition and test it.
	 */
	public static void sampleMain (){
		ColorSensor cs = new ColorSensor(SensorPort.S1);
		cs.setFloodlight(true);
		ObjectRecognition or = new ObjectRecognition(cs);
		
		or.calibrateBlueBlock();

		LCD.drawString("Press button", 0, 0);
		LCD.drawString("to query", 0, 1);
		
		while(Button.waitForAnyPress() != Button.ID_ESCAPE){
			LCD.clear();
			
			boolean isBlueBlock = or.checkColor();
			
			LCD.drawString("Press button", 0, 0);
			LCD.drawString("to query", 0, 1);
			LCD.drawString("Is blue: " + isBlueBlock, 0, 3);
		}
	}
	
	/**
	 * Constructor
	 * 
	 * @param cs The color sensor located on the claw
	 */
	public ObjectRecognition(ColorSensor cs){
		colorSensor = cs;
		calibratedBlueColor = new Vector(0.0, 0.0, 0.0);
	}
	
	
	/**
	 * Check the distance to an object, using the color sensor.
	 * 
	 * @return The distance from the sensor to the object
	 */
	public int checkDistance(){
		Color c = colorSensor.getColor();
		Vector v = new Vector(c.getRed(), c.getGreen(), c.getBlue());

		// Experimental formula
		double lightIntensity = v.length();
		double distance = Math.pow(350.0 / lightIntensity, 1 / 1.2);
		
		return (int)distance;
	}
	
	
	/**
	 * Checks the color of the object
	 * 
	 * @return True if the object is a blue block, false otherwise
	 */
	public Boolean checkColor(){
		Color c = colorSensor.getColor();
		
		Vector currentColor = new Vector(c.getRed(), c.getGreen(), c.getBlue());
		double cosTheta = Vector.cosTheta(currentColor, calibratedBlueColor);
		
		boolean isBlueBlock = (Math.abs(1.0 - cosTheta) < COS_THETA_MARGIN) &&
				              currentColor.length() > LENGTH_THRESHOLD;
		
		/*
		// Debugging information:
		// Current color
		LCD.drawString("+ " + (int)currentColor.getX() + ", " + (int)currentColor.getY() + ", " + (int)currentColor.getZ(), 0, 4);
		// Calibrated color
		LCD.drawString("+ " + (int)calibratedBlueColor.getX() + ", " + (int)calibratedBlueColor.getY() + ", " + (int)calibratedBlueColor.getZ(), 0, 5);
		// CosTheta
		LCD.drawString("$ " + cosTheta, 0, 6);
		// Theta
		LCD.drawString("% " + Math.acos(cosTheta) * 180.0 / Math.PI, 0, 7);
		*/
		
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
		
		LCD.clear();
		
		// This takes the ArrayList, transforms it to an array, and gets
		// the average color
		Vector colorArray[] = colors.toArray(new Vector[colors.size()]);
		calibratedBlueColor = Vector.average(colorArray);
	}
	
	/**
	 * Prints a message about the calibration.
	 * 
	 * Explains which buttons to press.
	 */
	private void printCalibrationMessage(){
		LCD.clear();
		LCD.drawString("Calibration", 0, 0);
		LCD.drawString("(Blue block)", 0, 1);
		LCD.drawString("Press to calibr.", 0, 3);
		LCD.drawString("Escape to stop", 0, 4);
	}
}
