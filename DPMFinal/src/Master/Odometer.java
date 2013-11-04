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

	private double RW_RADIUS = 2.1, LW_RADIUS = 2.1, WHEEL_BASE = 15.0;
	
	NXTRegulatedMotor leftMotor = Motor.A;
	NXTRegulatedMotor rightMotor = Motor.B;
	

	
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
	
	// accessors
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
	}
	
	
	//new tacho - old tacho = delta tacho
	public void updateTacho(){
		last_tacho_l = tacho_l;
		last_tacho_r = tacho_r;
		
		tacho_l = leftMotor.getTachoCount();
		tacho_r = rightMotor.getTachoCount();
		
		delta_tacho_l = tacho_l - last_tacho_l;
		delta_tacho_r = tacho_r - last_tacho_r;
	}
	
	//equation from tutorial slides
	public double calculateDeltaTheta(){
		double deltaTheta = ((delta_tacho_r * RW_RADIUS) - (delta_tacho_l * LW_RADIUS))/WHEEL_BASE;
		deltaTheta = deltaTheta * Math.PI/180; //needs to be in radians
		return deltaTheta;
	}
	
	public double calculateDeltaDistance(){
		double deltaDistance = ((delta_tacho_r * RW_RADIUS) + (delta_tacho_l * LW_RADIUS))/2;
		deltaDistance = deltaDistance * Math.PI/180;	//needs to be in radians
		return deltaDistance;
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

		return result * 180/Math.PI;
	}
	
	public void setX(double input) {
		
		synchronized (lock) {
			x = input;
		}

	}
	
	public void setY(double input) {
		
		synchronized (lock) {
			y = input;
		}

	}
	
	public void setTheta(double input) {
		synchronized (lock) {
			theta = input*Math.PI/180;
		}

	}
	
}

