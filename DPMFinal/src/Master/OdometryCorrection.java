package Master;

import lejos.nxt.ColorSensor.*;
import lejos.nxt.comm.RConsole;
import lejos.nxt.*;
//import lejos.nxt.ColorSensor.Color;
import lejos.util.Timer;

/**
 * Class to correct the robot's position and heading as it crosses black grid lines.
 * 
 * The position
 * 
 * @author Nick
 * @version 1.0
 */
public class OdometryCorrection extends Thread{
	enum Mode {WAITING, CORRECTING};
	
	// Constants
	private final Vector leftCSPosition = new Vector(-8.7, -10.5, 0.0);
	private final Vector rightCSPosition = new Vector(9.3, -10.5, 0.0);
	private final long CORRECTION_INTERVAL = 20;
	private final int BLACK_LINE_FILTER_SLOPE = -50;
	private final long TIME_THRESHOLD = 500;
	private final double MIN_DISTANCE_FROM_INTERSECTION = 3;
	private final double ERROR = 0.001;	// General double precision error
	
	// Variables
	private Navigation navigation;
	private Odometer odometer;
	private ColorSensor csLeft;
	private ColorSensor csRight;
	
	private SmoothDifferenceFilter filterLeft;
	private SmoothDifferenceFilter filterRight;
	
	private Boolean running;
	private Mode currentMode;
	
	// Each CS keeps track of the last X and Y grid lines it saw.
	private Vector lastXGridlineLeft;
	private Vector lastYGridlineLeft;
	private Vector lastXGridlineRight;
	private Vector lastYGridlineRight;
	
	// Keep track of whether you saw at least one grid line since the beginning.
	private Boolean sawXGridlineLeft;
	private Boolean sawYGridlineLeft;
	private Boolean sawXGridlineRight;
	private Boolean sawYGridlineRight;
	
	// Keep track of whether you just saw a new grid line.
	// (So that you'll only correct once).
	private Boolean newXGridlineLeft;
	private Boolean newYGridlineLeft;
	private Boolean newXGridlineRight;
	private Boolean newYGridlineRight;
	
	private long lastTimeGridlineLeft;
	private long lastTimeGridlineRight;
	
	private int numLeftLines = 0;
	private int numRightLines = 0;
	
	/**
	 * Constructor
	 * 
	 * @param odometer The robot's odometer.
	 * @param csLeft The left color sensor.
	 * @param csRight The right color sensor.
	 */
	public OdometryCorrection(Navigation navigation, Odometer odometer, ColorSensor csLeft, ColorSensor csRight){
		this.navigation = navigation;
		this.odometer = odometer;
		this.csLeft = csLeft;
		this.csRight = csRight;
		
		filterLeft = new SmoothDifferenceFilter(4);
		filterRight = new SmoothDifferenceFilter(4);
		
		running = false;
		
		lastXGridlineLeft = new Vector(0.0, 0.0, 0.0);
		lastXGridlineRight = new Vector(0.0, 0.0, 0.0);
		lastYGridlineLeft = new Vector(0.0, 0.0, 0.0);
		lastYGridlineRight = new Vector(0.0, 0.0, 0.0);
		
		currentMode = Mode.CORRECTING;
		
		csLeft.setFloodlight(Color.RED);
		csRight.setFloodlight(Color.RED);
		
		resetState();
	}
	
	/**
	 * Start the odometry correction.
	 * 
	 * The position and heading of the robot will be corrected.
	 */
	public void run(){
		running = true;
		currentMode = Mode.CORRECTING;
		
		while(running){
			long startTime = System.currentTimeMillis();
			
			//LCD.drawString("" + numLeftLines + "-L: " + csLeft.getNormalizedLightValue(), 0, 5);
			//LCD.drawString("" + numRightLines + "-R: " + csRight.getNormalizedLightValue(), 0, 6);
			
			// Do stuff
			execute();
			
			// Wait until end of loop time
			long timeTaken = System.currentTimeMillis() - startTime;
			long waitTime = CORRECTION_INTERVAL - timeTaken;
			if(waitTime > 0){
				try { Thread.sleep(waitTime); }
				catch (InterruptedException e) { e.printStackTrace(); }
			}
		}
	}
	
	/**
	 * Do a loop, where you check for gridlines and correct if you can.
	 */
	private void execute(){
		// Check the mode of operation
		//if(navigation.isObstacleAvoiding()){
		if(false){
			setMode(Mode.WAITING);
			return;
		}
		else{
			setMode(Mode.CORRECTING);
		}
		
		filterLeft.add(csLeft.getNormalizedLightValue());
		filterRight.add(csRight.getNormalizedLightValue());
		
		//LCD.drawString("L-filt " + filterLeft.getFilteredValue(), 0, 0);
		//LCD.drawString("R-filt " + filterRight.getFilteredValue(), 0, 1);
		//RConsole.println("L " + filterLeft.getRawValue() + " " + filterLeft.getFilteredValue() + " R " + filterRight.getRawValue() + " " + filterRight.getFilteredValue());
		
		//checkForNewGridlines();
		//checkForCorrection();
	}
	
	/**
	 * Check the CS to see if the robot is passing over a grid line.
	 */
	private void checkForNewGridlines(){
		checkForNewGridlineLeft();
		checkForNewGridlineRight();
	}
	
	/**
	 * Check if the left CS passed over a new grid line.
	 */
	private void checkForNewGridlineLeft(){
		//LCD.drawString("see L :" + seesLine(csLeft), 0, 3);
		//LCD.drawString("same L :" + sameLine(lastTimeGridlineLeft), 0, 4);
		if(seesLine(filterLeft)){
			if(!sameLine(lastTimeGridlineLeft)){
				numLeftLines++;
				Sound.beep();
				
				lastTimeGridlineLeft = System.currentTimeMillis();
				
				double odometerX = odometer.getX();
				double odometerY = odometer.getY();
				
				Vector odometerPosition = new Vector(odometerX, odometerY, 0.0);
				Vector gridPosition = Vector.add(odometerPosition,  leftCSPosition);
				
				double distanceX = distanceToNearestXGridline(gridPosition.getX());
				double distanceY = distanceToNearestYGridline(gridPosition.getY());
				
				if(Math.abs(distanceX) > Math.abs(distanceY) && Math.abs(distanceX) > MIN_DISTANCE_FROM_INTERSECTION){
					gridPosition.setY(nearestYGridline(odometerY));
					odometer.setY(odometerY + distanceY);

					sawYGridlineLeft = true;
					newYGridlineLeft = true;
					lastYGridlineLeft = gridPosition;
				}
				else if(Math.abs(distanceX) < Math.abs(distanceY) && Math.abs(distanceY) > MIN_DISTANCE_FROM_INTERSECTION){
					gridPosition.setX(nearestXGridline(odometerX));
					odometer.setX(odometerX + distanceX);

					sawXGridlineLeft = true;
					newXGridlineLeft = true;
					lastXGridlineLeft = gridPosition;
				}
			}
		}
	}
	
	/**
	 * Check if the right CS passed over a new grid line.
	 */
	private void checkForNewGridlineRight(){
		if(seesLine(filterRight)){
			if(!sameLine(lastTimeGridlineRight)){
				numRightLines++;
				Sound.beep();
				
				lastTimeGridlineRight = System.currentTimeMillis();
				
				double odometerX = odometer.getX();
				double odometerY = odometer.getY();
				
				Vector odometerPosition = new Vector(odometerX, odometerY, 0.0);
				Vector gridPosition = Vector.add(odometerPosition,  rightCSPosition);
				
				double distanceX = distanceToNearestXGridline(gridPosition.getX());
				double distanceY = distanceToNearestYGridline(gridPosition.getY());
				
				if(Math.abs(distanceX) > Math.abs(distanceY) && Math.abs(distanceX) > MIN_DISTANCE_FROM_INTERSECTION){
					gridPosition.setY(nearestYGridline(odometerY));
					odometer.setY(odometerY + distanceY);

					sawYGridlineRight = true;
					newYGridlineRight = true;
					lastYGridlineRight = gridPosition;
				}
				else if(Math.abs(distanceX) < Math.abs(distanceY) && Math.abs(distanceY) > MIN_DISTANCE_FROM_INTERSECTION){
					gridPosition.setX(nearestXGridline(odometerX));
					odometer.setX(odometerX + distanceX);

					sawXGridlineRight = true;
					newXGridlineRight = true;
					lastXGridlineRight = gridPosition;
				}
			}
		}
	}
	
	/**
	 * Try to correct position or heading, if it has read new grid lines.
	 */
	private void checkForCorrection(){
		// Position correction is already done inside checkForNewGridline***()
		
		checkForXCorrection();
		checkForYCorrection();
	}
	
	/**
	 * Correct the heading if the robot recently completely passed over an
	 * x grid line.
	 */
	private void checkForXCorrection(){
		// Check that both sensor have seen at least one line.
		if(sawXGridlineLeft && sawXGridlineRight){
			if(newXGridlineLeft){
				// Check that the two x grid lines are from the same line.
				double xDifference = Vector.substract(lastXGridlineLeft, lastXGridlineRight).getX();
				
				if(xDifference < ERROR){
					Vector firstPos = Vector.add(lastXGridlineRight, rightCSPosition);
					Vector secondPos = Vector.add(lastXGridlineLeft, leftCSPosition);
					Vector delta = Vector.substract(secondPos, firstPos);
					double distance = delta.length();
					
					double clockwiseFrom90 = Math.atan(BTSendController.WHEELBASE_WIDTH / distance) * (180.0 / Math.PI);
					double angle = 90 - clockwiseFrom90;
					angle = Odometer.fixDegAngle(angle);
					
					// The robot could have crossed the line with this angle, or
					// the opposite angle.
					double angleAwayFromHeading = odometer.getTheta() - angle;
					angleAwayFromHeading = Odometer.fixDegAngle(angleAwayFromHeading);
					if(angleAwayFromHeading > 180.0){
						angleAwayFromHeading -= 180.0;
					}
					
					if(angleAwayFromHeading < 90.0){
						odometer.setTheta(angle);
					}
					else{
						odometer.setTheta(angle + 180.0);
					}
				}
				
				newXGridlineLeft = false;
				newXGridlineRight = false;
			}
			else if(newXGridlineRight){
				// Check that the two x grid lines are from the same line.
				double xDifference = Vector.substract(lastXGridlineLeft, lastXGridlineRight).getX();
				
				if(xDifference < ERROR){
					Vector firstPos = Vector.add(lastXGridlineRight, rightCSPosition);
					Vector secondPos = Vector.add(lastXGridlineLeft, leftCSPosition);
					Vector delta = Vector.substract(secondPos, firstPos);
					double distance = delta.length();
					
					double counterclockwiseFrom270 = Math.atan(BTSendController.WHEELBASE_WIDTH / distance) * (180.0 / Math.PI);
					double angle = 90 - counterclockwiseFrom270;
					angle = Odometer.fixDegAngle(angle);
					
					// The robot could have crossed the line with this angle, or
					// the opposite angle.
					double angleAwayFromHeading = odometer.getTheta() - angle;
					angleAwayFromHeading = Odometer.fixDegAngle(angleAwayFromHeading);
					if(angleAwayFromHeading > 180.0){
						angleAwayFromHeading -= 180.0;
					}
					
					if(angleAwayFromHeading < 90.0){
						odometer.setTheta(angle);
					}
					else{
						odometer.setTheta(angle + 180.0);
					}
				}
				
				newXGridlineLeft = false;
				newXGridlineRight = false;
			}
		}
	}
	
	private void checkForYCorrection(){
		// Check that both sensor have seen at least one line.
		if(sawYGridlineLeft && sawYGridlineRight){
			if(newYGridlineLeft){
				// Check that the two x grid lines are from the same line.
				double yDifference = Vector.substract(lastYGridlineLeft, lastYGridlineRight).getY();
				
				if(yDifference < ERROR){
					Vector firstPos = Vector.add(lastYGridlineRight, rightCSPosition);
					Vector secondPos = Vector.add(lastYGridlineLeft, leftCSPosition);
					Vector delta = Vector.substract(secondPos, firstPos);
					double distance = delta.length();
					
					double clockwiseFrom90 = Math.atan(BTSendController.WHEELBASE_WIDTH / distance) * (180.0 / Math.PI);
					double angle = 90 - clockwiseFrom90;
					angle = Odometer.fixDegAngle(angle);
					
					// The robot could have crossed the line with this angle, or
					// the opposite angle.
					double angleAwayFromHeading = odometer.getTheta() - angle;
					angleAwayFromHeading = Odometer.fixDegAngle(angleAwayFromHeading);
					if(angleAwayFromHeading > 180.0){
						angleAwayFromHeading -= 180.0;
					}
					
					if(angleAwayFromHeading < 90.0){
						odometer.setTheta(angle);
					}
					else{
						odometer.setTheta(angle + 180.0);
					}
				}
				
				newYGridlineLeft = false;
				newYGridlineRight = false;
			}
			else if(newYGridlineRight){
				// Check that the two x grid lines are from the same line.
				double yDifference = Vector.substract(lastYGridlineLeft, lastYGridlineRight).getY();
				
				if(yDifference < ERROR){
					Vector firstPos = Vector.add(lastYGridlineRight, rightCSPosition);
					Vector secondPos = Vector.add(lastYGridlineLeft, leftCSPosition);
					Vector delta = Vector.substract(secondPos, firstPos);
					double distance = delta.length();
					
					double counterclockwiseFrom270 = Math.atan(BTSendController.WHEELBASE_WIDTH / distance) * (180.0 / Math.PI);
					double angle = 90 - counterclockwiseFrom270;
					angle = Odometer.fixDegAngle(angle);
					
					// The robot could have crossed the line with this angle, or
					// the opposite angle.
					double angleAwayFromHeading = odometer.getTheta() - angle;
					angleAwayFromHeading = Odometer.fixDegAngle(angleAwayFromHeading);
					if(angleAwayFromHeading > 180.0){
						angleAwayFromHeading -= 180.0;
					}
					
					if(angleAwayFromHeading < 90.0){
						odometer.setTheta(angle);
					}
					else{
						odometer.setTheta(angle + 180.0);
					}
				}
				
				newYGridlineLeft = false;
				newYGridlineRight = false;
			}
		}
	}
	
	/**
	 * Change mode of operation.
	 * 
	 * Basically whether it is trying to correct or not.
	 * 
	 * @param newMode The new mode of operation.
	 */
	public void setMode(Mode newMode){
		if(currentMode != newMode){
			if(newMode == Mode.WAITING){
				resetState();
			}
		}
		
		currentMode = newMode;
	}
	
	/**
	 * Check whether the CS sees a black line.
	 * 
	 * @param cs The color sensor
	 * 
	 * @return True if the CS sees a black line, false otherwise.
	 */
	private Boolean seesLine(SmoothDifferenceFilter filter){
		return filter.getFilteredValue() < BLACK_LINE_FILTER_SLOPE;
	}
	
	/**
	 * Check if the line read by the CS is the same as a previous line.
	 * 
	 * @param lastTime The Last time a line was read.
	 * 
	 * @return True if it's the same line, false otherwise.
	 */
	private Boolean sameLine(long firstTime){
		return System.currentTimeMillis() - firstTime < TIME_THRESHOLD;
	}
	
	/**
	 * Reset the state of the odometry correction.
	 * 
	 * It will forget about any grid lines it passed.
	 */
	private void resetState(){
		lastXGridlineLeft.set(0.0, 0.0, 0.0);
		lastYGridlineLeft.set(0.0, 0.0, 0.0);
		lastXGridlineRight.set(0.0, 0.0, 0.0);
		lastYGridlineRight.set(0.0, 0.0, 0.0);
		
		sawXGridlineLeft = false;
		sawYGridlineLeft = false;
		sawXGridlineRight = false;
		sawYGridlineRight = false;

		newXGridlineLeft = false;
		newYGridlineLeft = false;
		newXGridlineRight = false;
		newYGridlineRight = false;

		lastTimeGridlineLeft = 0;
		lastTimeGridlineRight = 0;
	}
	
	/**
	 * Find the nearest x grid line.
	 * 
	 * @param x The x coordinate.
	 * 
	 * @return The x coordinate of the nearest x grid line.
	 */
	private double nearestXGridline(double x){
		return Math.floor((x + 15.0) / 30.0) * 30.0;
	}

	
	/**
	 * Find the nearest y grid line.
	 * 
	 * @param x The y coordinate.
	 * 
	 * @return The y coordinate of the nearest y grid line.
	 */
	private double nearestYGridline(double y){
		return Math.floor((y + 15.0) / 30.0) * 30.0;
	}

	
	/**
	 * Find the distance to the nearest x grid line.
	 * 
	 * You just need to add this correction to your x to get the grid line.
	 * 
	 * @param x The x coordinate.
	 * 
	 * @return The x coordinate of the nearest x grid line.
	 */
	private double distanceToNearestXGridline(double x){
		return nearestXGridline(x) - x;
	}

	
	/**
	 * Find the distance to the nearest y grid line.
	 * 
	 * You just need to add this correction to your y to get the grid line.
	 * 
	 * @param y The y coordinate.
	 * 
	 * @return The y coordinate of the nearest y grid line.
	 */
	private double distanceToNearestYGridline(double y){
		return nearestYGridline(y) - y;
	}
}
