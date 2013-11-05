package Slave;

import lejos.nxt.ColorSensor;

public class ObjectRecognition {
	
	private ColorSensor sens;

	/**
	 * Constructor
	 * @param cs The color sensor located on the claw
	 */
	public ObjectRecognition(ColorSensor cs){
		sens = cs;
	}
	
	
	/**
	 * Check the distance from the sensor to the object, may lead to signal being sent
	 * back to master to reposition robot in front of the block
	 * @return int - The distance from the sensor to the object
	 */
	public int checkDistance(){
		return 0;
	}
	
	
	/**
	 * Checks the color of the object
	 * @return True if the object is a blue block, false otherwise
	 */
	public Boolean checkColor(){
		return false;
	}
}
