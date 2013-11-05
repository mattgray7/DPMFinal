package Master;
import lejos.nxt.*;

public class Localization {
	
	private UltrasonicSensor us;
	
	/**
	 * Constructor
	 * @param odom	The shared odometer amongst classes
	 * @param US	The ultrasonic sensor used for localizing
	 */
	public Localization(Odometer odom, UltrasonicSensor US){
		us = US;
	}
	
	
	/**
	 * Will perform either falling or rising edge localization depending on if it sees a wall initally
	 * @return void
	 */
	public void doLocalization(){
		
	}
	
	
	/**
	 * Determines if the wall is within the distance threshold
	 * @return True	if the wall is within threshold
	 * @return False if the wall is past the threshold
	 */
	public Boolean wallInSight(){
		
		return false;
	}
	
	
	/**
	 * Clips ultrasonic readings to only accept values within near the first tile
	 * @return int - The clipped distance
	 */
	public int getFilteredData(){
		
		return 0;
	}
	
}
