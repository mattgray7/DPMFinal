package Master;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.util.Timer;
import lejos.util.TimerListener;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;


/*
 * Keeps track of the robot's position, using the displacement of its wheels.
 * 
 * @author Julian Liberta
 * @author Nicholas Aird
 */
public class Odometer extends Thread {
	// Constants
	private static final long ODOMETER_PERIOD = 25;	// Odometer's update period
	
	private double leftWheelRadius;
	private double rightWheelRadius;
	private double wheelBaseWidth;
	
	private double x;
	private double y;
	private double theta;
	
	private double masterDistance = 0;
	
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	
	private long lastTachoLeft;
	private long lastTachoRight;
	
	private long lastTime;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(double leftRadius, double rightRadius, double width) {
		leftMotor = Motor.B;
		rightMotor = Motor.C;
		
		x = 0.0;
		y = 0.0;
		theta = 90.0;
		lock = new Object();
		
		leftWheelRadius = leftRadius;
		rightWheelRadius = rightRadius;
		wheelBaseWidth = width;
		
		lastTachoLeft = leftMotor.getTachoCount();
		lastTachoRight = rightMotor.getTachoCount();
	}

	/**
	 * The main method that constantly updates the odometer
	 */
	public void run() {
		// Reset some variables (this is done only once, at the beginning)
		lastTachoLeft = leftMotor.getTachoCount();
		lastTachoRight = rightMotor.getTachoCount();
		
		// Initialize some other variables
		long startTime = System.currentTimeMillis();
		lastTime = startTime;
		
		// Continuously update the odometer's state
		while (true) {
			startTime = System.currentTimeMillis();
			
			// Current tachometers for each motor
			long currentTachoLeft = leftMotor.getTachoCount();
			long currentTachoRight = rightMotor.getTachoCount();
			
			// Change in wheel rotation angle since last update, in degrees.
			double deltaTachoLeft = (currentTachoLeft - lastTachoLeft) * Math.PI / 180.0;
			double deltaTachoRight = (currentTachoRight - lastTachoRight) * Math.PI / 180.0;
			
			// Change in the robot's heading
			double deltaTheta = (deltaTachoRight * rightWheelRadius - deltaTachoLeft * leftWheelRadius) * 180.0 / (Math.PI * wheelBaseWidth);
			
			// Distance traveled by the robot's center
			double distance = (deltaTachoLeft * leftWheelRadius + deltaTachoRight * rightWheelRadius) / 2.0;
			masterDistance += distance;
			double dx = distance * Math.cos((theta + deltaTheta / 2.0) * Math.PI / 180.0);
			double dy = distance * Math.sin((theta + deltaTheta / 2.0) * Math.PI / 180.0);
			
			// Update odometer's state.
			synchronized (lock) {
				x += dx;
				y += dy;
				theta += deltaTheta;
				theta = fixDegAngle(theta);	// Can't be done outside
				
				// If you wanted to do it outside, you'd need a synchronized
				// block to access theta. That would defeat the purpose (we
				// want less stuff done inside synchronized blocks!)
			}
			
			// Reset some variables for the next loop.
			lastTime = startTime;
			lastTachoLeft = currentTachoLeft;
			lastTachoRight = currentTachoRight;
			
			// Waste some time to respect the odometer's period.
			long endTime = System.currentTimeMillis();
			long loopTime = endTime - startTime;
			if (loopTime < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - loopTime);
				} catch (InterruptedException e) {
					// Don't care.
				}
			}
		}
	}
	
	/**
	 * Get the state of the odometer.
	 * 
	 * @param position A "double" array (of size 3) to hold the x, y and theta
	 * @param update A "boolean" array (of size 3) to state what state you want.
	 */
	public void getPosition(double[] position, boolean[] update) {
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	/**
	 * Get the x coordinate.
	 * 
	 * @return The x coordinate.
	 */
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	/**
	 * Get the y coordinate.
	 * 
	 * @return The y coordinate.
	 */
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	/**
	 * Get the heading.
	 * 
	 * @return The heading (in degrees).
	 */
	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	/**
	 * Set the odometer's state.
	 * 
	 * @param position A "double" array (of size 3) to hold the x, y and theta.
	 * @param update A "boolean" array (of size 3) to say what state you want to change.
	 */
	public void setPosition(double[] position, boolean[] update) {
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	/**
	 * Set the x coordinate.
	 * 
	 * @return The x coordinate.
	 */
	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	/**
	 * Set the y coordinate.
	 * 
	 * @return The y coordinate.
	 */
	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	/**
	 * Get the heading.
	 * 
	 * @return The heading (in degrees).
	 */
	public void setTheta(double theta) {
		theta = fixDegAngle(theta);
		
		synchronized (lock) {
			this.theta = theta;
		}
	}

	/**
	 * Returns the equivalent angle in the range [0.0, 360.0].
	 * 
	 * @param angle The angle (can be negative or greater than 360.0).
	 * 
	 * @return The angle in the range [0.0, 360.0].
	 */
	public static double fixDegAngle(double angle) {
		// Ensure that the angle is positive
		while(angle < 0.0){
			angle += 360.0;
		}
		
		// Snap it to the range [0.0, 360.0]
		angle %= 360.0;
		
		return angle;
	}
	
	/**
	 * Get the left wheel's radius (in cm).
	 * 
	 * @return The left wheel's radius (in cm).
	 */
	public double getLeftRadius(){
		return this.leftWheelRadius;
	}
	
	/**
	 * Get the right wheel's radius (in cm).
	 * 
	 * @return The right wheel's radius (in cm).
	 */
	public double getRightRadius(){
		return this.rightWheelRadius;
	}
	
	/**
	 * Get the wheel base's width (in cm).
	 * 
	 * @return The wheel base's width (in cm).
	 */
	public double getWheelBase(){
		return this.wheelBaseWidth;
	}
	
	public double getMasterDistance(){
		return this.masterDistance;
	}
}
