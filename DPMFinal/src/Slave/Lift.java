

package Slave;
import lejos.nxt.*;

public class Lift {
	private static NXTRegulatedMotor armMotor = Motor.A;
	private static NXTRegulatedMotor clampMotor = Motor.B;

	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		
		//lowerArms(450);
		//raiseArms(455);
		
		clamp();
		raiseArms(300);
		
		try {
			Thread.sleep(5000);
		} catch (InterruptedException e) {
		}

	}
	
	
	/**
	 * Constructor
	 * @param arm The arm motor of the lifting mechanism
	 * @param clamp The clamp motor that controls the claws
	 */
	public Lift(NXTRegulatedMotor arm, NXTRegulatedMotor clamp){
		armMotor = arm;
		clampMotor = clamp;
	}
	
	
	/**
	 * Lowers the mechanical arms by an input angle
	 * @param angle The angle in which to rotate the arm
	 * @return void
	 */
	public static void lowerArms(int angle){
		armMotor.setSpeed(200);
		armMotor.backward();
		armMotor.rotate(-angle, false);
		armMotor.stop();
	}
	
	
	/**
	 * Raises the mechanical arms by an input angle
	 * @param angle The angle in which to rotate the arm
	 * @return void
	 */
	public static void raiseArms(int angle){
		armMotor.setSpeed(300);
		armMotor.forward();
		armMotor.rotate(angle, false);
		armMotor.stop();
		
		
	}
	
	
	/**
	 * Clamps the claws together
	 * @return void
	 */
	public static void clamp(){
		clampMotor.setSpeed(100);
		clampMotor.backward();
		clampMotor.rotate(-95, false);
	}
	
	
	/**
	 * Opens the claws
	 * @return void
	 */
	public static void release(){
		/*clampMotor.setSpeed(100);
		clampMotor.forward();
		
		try { Thread.sleep(200); }catch (InterruptedException e) {}
		clampMotor.stop();*/
	}
	
	
	/**
	 * Tentative - if using an ultrasonic sensor, method will read the height of the tower in front of it
	 * @return int - The height of the tower
	 */
	public int getTowerHeight(){
		return 0;
	}
	
	
	/**
	 * Only used by garbage collector. Will knock down tower if outside of opponent's green zone
	 * @return void
	 */
	public void destroyTower(){
		
	}
	
	/*
	 * These methods are commented as I don't they will be included in the final system. They involve sending
	 * messages back and forth with the master brick to reposition the robot to be in a better position to lift the brick.
	 * If our clamps grab it well enough at any orientation adn the robot can be positioned well before
	 * the control is switched over, these methods should be unnecessary.
	 */
	/*
	//will send signal to master to better position the robot
	public void positionRobot(){
		
	}
	
	//will rotate(?) sensor to get orientation of block, may lead to repositioning the robot to grab block
	//currently our mechanical design clamps rather well regardless of the blocks orientation
	public void getBlockOrientation(){
		
	}*/
}
