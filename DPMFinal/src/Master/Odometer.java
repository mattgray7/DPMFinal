package Master;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class Odometer implements TimerListener {
	public static final int DEFAULT_PERIOD = 25;
	private Timer odometerTimer;
//	private Navigation nav;
	// position data
	private Object lock;
	public static double x, y, theta;
	private int last_tacho_l, last_tacho_r;
	private int tacho_l, tacho_r;
	private int delta_tacho_l, delta_tacho_r;

	private double RW_RADIUS = 2.0, LW_RADIUS = 2.0, WHEEL_BASE = 16.0;
	
	NXTRegulatedMotor leftMotor = Motor.A;
	NXTRegulatedMotor rightMotor = Motor.B;
	

	/**
	 * Constructor
	 * @param period How often the odometer should be updated
	 * @param start True if the odometer should start immediately
	 */
	public Odometer(int period, boolean start) {
		// initialise variables

		//this.nav = new Navigation(this);
		odometerTimer = new Timer(period, this);
		x = 0.0;
		y = 0.0;
		theta = Math.PI/2;
		lock = new Object();
		
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		
		tacho_l = leftMotor.getTachoCount();
		tacho_r = rightMotor.getTachoCount();
		
		// start the odometer immediately, if necessary
		if (start)
			odometerTimer.start();
	}
	
	/**
	 * Update Odometer at every clock tick
	 * @return void
	 */
	public void timedOut() {
		
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			
			//get delta tacho
			updateTacho();
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				
				//get delta theta from delta tacho count
				double deltaTheta = calculateDeltaTheta();
				theta = theta + deltaTheta;
				
				//formulas for x and y from tutorial slides
				double deltaDistance = calculateDeltaDistance();
				x = getX() + deltaDistance*Math.cos(theta + deltaTheta/2);
				y = getY() + deltaDistance*Math.sin(theta + deltaTheta/2);
			}
		}
	}
	
	/*
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}

	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) theta = pos[2];
		}
	}*/
	
	
	/**
	 * Updates the tachometer count for each motor shaft
	 * @return false
	 */
	public void updateTacho(){
		last_tacho_l = tacho_l;
		last_tacho_r = tacho_r;
		
		tacho_l = leftMotor.getTachoCount();
		tacho_r = rightMotor.getTachoCount();
		
		delta_tacho_l = tacho_l - last_tacho_l;
		delta_tacho_r = tacho_r - last_tacho_r;
	}
	
	/**
	 * Calculates the change in the odometer's orientation
	 * @return double - The robot's orientation change
	 */
	public double calculateDeltaTheta(){
		double deltaTheta = ((delta_tacho_r * RW_RADIUS) - (delta_tacho_l * LW_RADIUS))/WHEEL_BASE;
		deltaTheta = deltaTheta * Math.PI/180; //needs to be in radians
		return deltaTheta;
	}
	
	/**
	 * Calculates the change in the odometer's position
	 * @return double - The robot's position change
	 */
	public double calculateDeltaDistance(){
		double deltaDistance = ((delta_tacho_r * RW_RADIUS) + (delta_tacho_l * LW_RADIUS))/2;
		deltaDistance = deltaDistance * Math.PI/180;	//needs to be in radians
		return deltaDistance;
	}
	
	/**
	 * @return double - The x value of the robot
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * @return double - The y value of the robot
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}
	
	/**
	 * @return double - The theta value of the robot
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result * 180/Math.PI;
	}
	
	/**
	 * @param input The new x value of the robot
	 * @return void
	 */
	public void setX(double input) {
		
		synchronized (lock) {
			x = input;
		}

	}
	
	/**
	 * @param input The new y value of the robot
	 * @return void
	 */
	public void setY(double input) {
		
		synchronized (lock) {
			y = input;
		}

	}
	
	/**
	 * @param input The new theta value of the robot
	 * @return void
	 */
	public void setTheta(double input) {
		synchronized (lock) {
			theta = input*Math.PI/180;
		}

	}
	
}

