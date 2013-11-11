import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.ColorSensor;
import lejos.nxt.comm.RConsole;
import lejos.robotics.Color;

/**
 * Displays the value of the color sensor's getColor() method to the
 * brick's screen. It can also send the output to a pc via Bluetooth.
 * 
 * @author Nicholas Aird
 * @version 1.0
 */
public class BasicCS {
	static SensorPort usPort = SensorPort.S1;
	static int BT_Timeout = 10000;	// 10 seconds
	
	public static void main(String args[]){
		ColorSensor cs = new ColorSensor(usPort);
		cs.setFloodlight(true);
		
		setupBluetooth();
		
		int buttonsPressed = 0;
		while( !((buttonsPressed & Button.ID_ESCAPE) > 0) ){
			Color c = cs.getColor();
			
			// Print to screen
			LCD.clear();
			String messageA = "RGB:";
			String messageB = "" + c.getRed();
			String messageC = "" + c.getGreen();
			String messageD = "" + c.getBlue();
			LCD.drawString(messageA, 0, 0);
			LCD.drawString(messageB, 0, 1);
			LCD.drawString(messageC, 0, 2);
			LCD.drawString(messageD, 0, 3);
			RConsole.println(messageA + " " + messageB + ", " + messageC + ", " + messageD);
			
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
