import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.ColorSensor;
import lejos.nxt.comm.RConsole;
import lejos.robotics.Color;


/**
 * Time it takes for the CS to do one getColor().
 * 
 * This class finds the amount of time it takes for the color sensor to
 * execute a single getColor(). It then displays the value to the brick's screen
 * or sends it to a pc via bluetooth.
 * 
 * @author Nicholas Aird
 * @version 1.0
 */
public class CSColorTime {
	static SensorPort usPort = SensorPort.S1;
	static int BT_TIMEOUT = 10000;	// 10 seconds
	static int NUMBER_OF_QUERIES = 10;
	
	public static void main(String args[]){
		ColorSensor cs = new ColorSensor(usPort);
		cs.setFloodlight(true);
		
		setupBluetooth();
		
		// Wait for start
		LCD.drawString("Press button", 0, 0);
		LCD.drawString("to start", 0, 1);
		Button.waitForAnyPress();
		
		do{
			LCD.clear();
			LCD.drawString("Testing...", 0, 0);
			
			performQueryTest(cs);
			
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
	 * Finds the amount of time for one getColor() and displays it to the screen.
	 */
	public static void performQueryTest(ColorSensor cs){
		double timePerQuery = getTimePerQuery(cs);
		
		String queryTimeMessageA = "ms/getColor():";
		String queryTimeMessageB = "" + timePerQuery;
		LCD.drawString(queryTimeMessageA, 0, 2);
		LCD.drawString(queryTimeMessageB, 0, 3);
		RConsole.println(queryTimeMessageA + " " + queryTimeMessageB);
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
	 * Find the amount of time it takes to execute the color sensor's
	 * getColor().
	 * 
	 * This also includes the amount of time it takes to execute java's for
	 * loop.
	 * 
	 * @param numberOfLoops The number of times the loop will iterate.
	 * @param us The color sensor.
	 * @return The time in ms to execute all the loops and getColor()'s.
	 */
	public static long getTotalLoopAndQueryTime(int numberOfLoops, ColorSensor cs){
		long startTime = System.currentTimeMillis();
		for(int i = 0; i < numberOfLoops; i++){
			cs.getColor();
		}
		long endTime = System.currentTimeMillis();
		
		return endTime - startTime;
	}
	
	/**
	 * Find the amount of time it takes the color sensor to execute
	 * one getColor().
	 * 
	 * @return Time (in ms) it takes for the CS to do one getColor().
	 */
	public static double getTimePerQuery(ColorSensor cs){
		int numberOfQueries = NUMBER_OF_QUERIES;
		
		long totalLoopTime = getTotalLoopTime(numberOfQueries);
		long totalLoopAndQueryTime = getTotalLoopAndQueryTime(numberOfQueries, cs);
		
		long totalQueryTime = totalLoopAndQueryTime - totalLoopTime;
		return totalQueryTime / (double)numberOfQueries;
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
