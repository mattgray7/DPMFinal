import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;

/**
 * Displays the value of the ultrasonic sensor's getDistance() method to the
 * brick's screen. It can also send the output to a pc via Bluetooth.
 * 
 * @author Nicholas Aird
 * @version 1.0
 */
public class BasicUS {
	static SensorPort usPort = SensorPort.S1;
	static int BT_Timeout = 10;	// 10 seconds
	
	public static void main(String args[]){
		UltrasonicSensor us = new UltrasonicSensor(usPort);
		
		setupBluetooth();
		
		int buttonsPressed = 0;
		while( !((buttonsPressed & Button.ID_ESCAPE) > 0) ){
			int distance = us.getDistance();
			
			// Print to screen
			LCD.clear();
			LCD.drawString("Distance: " + distance, 0, 0);
			RConsole.println("Distance: " + distance);
			
			// Update currently pressed buttons
			buttonsPressed = Button.readButtons();
		}
		
		closeBluetooth();
	}

	/**
	 * Asks the user whether to open Bluetooth or not.
	 * 
	 * If Bluetooth is chosen, the brick will search for a bluetooth
	 * connection until a certain maximum amount of time elapses.
	 */
	public static void setupBluetooth(){
		LCD.clear();
		
		LCD.drawString("BT    |  no BT", 0, 0);
		int choice = Button.waitForAnyPress();
		if(choice == Button.ID_LEFT){
			RConsole.openBluetooth(BT_Timeout);
		}
		else{
			// Don't open bluetooth
		}
		
		LCD.clear();
	}
	
	/**
	 * Close the Bluetooth connection
	 */
	public static void closeBluetooth(){
		if(RConsole.isOpen()){
			RConsole.close();
		}
	}
}
