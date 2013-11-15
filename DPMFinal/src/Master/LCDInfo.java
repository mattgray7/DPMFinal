package Master;

import lejos.nxt.LCD;
import lejos.util.Timer;
import lejos.util.TimerListener;

/**
 * Class to continuously display the odometer's state.
 * 
 * Calling the constructor will start an internal timer. It will then
 * print out the odometer's state continuously.
 * 
 * @author Matt
 *
 */
public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer timer;
	
	/**
	 * Constructor.
	 * 
	 * @param odo The shared odometer amongst classes.
	 */
	public LCDInfo(Odometer odo) {
		this.odo = odo;
		this.timer = new Timer(LCD_REFRESH, this);
		
		this.timer.start();
	}
	
	/**
	 * Display the odometer's position every clock tick.
	 * 
	 * @return void
	 */
	public void timedOut() { 
		LCD.clear();
		LCD.drawString("X: " + (int)odo.getX(), 0, 0);
		LCD.drawString("Y: " + (int)odo.getY(), 0, 1);
		LCD.drawString("H: " + (int)odo.getTheta(), 0, 2);
	}
}

