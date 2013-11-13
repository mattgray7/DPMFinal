package Master;

import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.util.Timer;
import lejos.util.TimerListener;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;

/*
 * Odometer.java
 */
//Code written by Group 21
public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	//Declare other variables necessary in this class
	//Current tachometers for each motor
	private double tachoB;
	private double tachoC;
	//Previous tachometers for each motor
	private double previousTachoB;
	private double previousTachoC;
	//Change in tachometers
	private double deltaTachoB;
	private double deltaTachoC;
	//Angular displacement
	private double deltaTheta;
	//Arclength of travelled path
	private double arcLength;
	//Wheel radius
	private double leftRadius;
	private double rightRadius;
	//Center-to-center wheel distance
	private double width;
	private double pi = Math.PI;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(double leftRadius, double rightRadius, double width) {
		x = -15.0;
		y = 0.0;
		theta = 90.0;
		lock = new Object();
		//Left and right tachometer changes start at 0
		deltaTachoB = 0;
		deltaTachoC = 0;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
		previousTachoB = 0;
		previousTachoC = 0;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			//Get current tachometer for each motor
			tachoB = Motor.B.getTachoCount();
			tachoC = Motor.C.getTachoCount();
			//Calculate the tachometer value of each motor for each interval of movement
			//Convert to radians!
			deltaTachoB = (tachoB - previousTachoB)*pi/180;
			deltaTachoC = (tachoC - previousTachoC)*pi/180;
			//calculate change in angle
			deltaTheta = (deltaTachoC*rightRadius - deltaTachoB*leftRadius)*180/(pi*width);
			//Calculate arc length
			arcLength = (deltaTachoB*leftRadius + deltaTachoC*rightRadius)/2;
			synchronized (lock) {
				//Update current x and y coordinates of the robot
				//Update angle
				x += arcLength*Math.cos((theta + deltaTheta/2)*pi/180);
				y += arcLength*Math.sin((theta + deltaTheta/2)*pi/180);
				theta += deltaTheta;
				//theta = (fixDegAngle(theta));
			}
			//Set current tachometers to the previous tachometers and repeat loop
			previousTachoB = tachoB;
			previousTachoC= tachoC;
			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}

	public static double fixDegAngle(double angle) {
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);

		return angle % 360.0;
	}
	
	public double getLeftRadius(){
		return this.leftRadius;
	}
	
	public double getWheelBase(){
		return this.width;
	}
	
	public double getRightRadius(){
		return this.rightRadius;
	}

}