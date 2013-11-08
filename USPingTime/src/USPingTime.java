import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;


/**
 * Time it takes for the US to do one ping().
 * 
 * This class finds the amount of time it takes for the ultrasonic sensor to
 * execute a single ping(). It then displays the value to the brick's screen
 * or sends it to a pc via bluetooth.
 * 
 * @author Nicholas Aird
 * @version 1.0
 */
public class USPingTime {
	static SensorPort usPort = SensorPort.S1;
	static int BT_TIMEOUT = 10;	// 10 seconds
	static int NUMBER_OF_PINGS = 100;
	
	public static void main(String args[]){
		UltrasonicSensor us = new UltrasonicSensor(usPort);
		
		setupBluetooth();
		
		// Wait for start
		LCD.drawString("Press button", 0, 0);
		LCD.drawString("to start", 0, 1);
		Button.waitForAnyPress();
		
		do{
			LCD.clear();
			performPingTest(us);
			
			// Ask for another iteration
			LCD.drawString("Again  |   Done", 0, 0);
		} while(Button.waitForAnyPress() == Button.ID_LEFT);
		
		LCD.clear();
		LCD.drawString("Press any", 0, 4);
		LCD.drawString("button to quit", 0, 5);
		Button.waitForAnyPress();
		
		closeBluetooth();
	}
	
	/**
	 * Finds the amount of time for one ping and displays it to the screen.
	 */
	public static void performPingTest(UltrasonicSensor us){
		long timePerPing = getTimePerPing(us);
		
		String pingTimeMessage = "ms/ping: " + timePerPing;
		LCD.drawString(pingTimeMessage, 0, 2);
		RConsole.println(pingTimeMessage);
	}
	
	/**
	 * Find the amount of time it takes to execute java's for loop.
	 * 
	 * @param numberOfLoops The number of times the loop will iterate.
	 * 
	 * @return The time (in ms) to execute all the loops.
	 */
	public static long getTotalLoopTime(int numberOfLoops){
		long startTime = System.currentTimeMillis();
		for(int i = 0; i < numberOfLoops; i++){
			// Do nothing
		}
		long endTime = System.currentTimeMillis();
		
		return endTime - startTime;
	}
	
	/**
	 * Find the amount of time it takes to execute the ultrasonic sensor's
	 * ping.
	 * 
	 * This also includes the amount of time it takes to execute java's for
	 * loop.
	 * 
	 * @param numberOfLoops The number of times the loop will iterate.
	 * @param us The ultrasonic sensor.
	 * @return The time in ms to execute all the loops and pings
	 */
	public static long getTotalLoopAndPingTime(int numberOfLoops, UltrasonicSensor us){
		long startTime = System.currentTimeMillis();
		for(int i = 0; i < numberOfLoops; i++){
			us.getDistance();
			// us.ping();	// Is this the same?
		}
		long endTime = System.currentTimeMillis();
		
		return endTime - startTime;
	}
	
	/**
	 * Find the amount of time it takes the ultrasonic sensor to execute
	 * one ping().
	 * 
	 * @return Time (in ms) it takes for the US to do one ping().
	 */
	public static long getTimePerPing(UltrasonicSensor us){
		int numberOfPings = NUMBER_OF_PINGS;
		
		long totalLoopTime = getTotalLoopTime(numberOfPings);
		long totalLoopAndPingTime = getTotalLoopAndPingTime(numberOfPings, us);
		
		long totalPingTime = totalLoopAndPingTime - totalLoopTime;
		return totalPingTime / numberOfPings;
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
			RConsole.openBluetooth(BT_TIMEOUT);
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
