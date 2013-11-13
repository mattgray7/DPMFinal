

package Slave;
import lejos.nxt.*;

public class Lift {
	private static NXTRegulatedMotor armMotor;
	private static NXTRegulatedMotor clampMotor;

	/*
	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		//lowerArms();
		clamp();
		raiseArms();
		lowerArms();
		release();
		//new commit test
		
		try { Thread.sleep(2000); }catch (InterruptedException e) {}
		//release();
		//lowerArms();
	}
	*/
	
	
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
	public void lowerArms(int angle){
		/*Sound.buzz();
		
		armMotor.setSpeed(200);
		armMotor.backward();
		
		try { Thread.sleep(500); }catch (InterruptedException e) {}
		
		armMotor.stop();
		Sound.buzz();*/
	}
	
	
	/**
	 * Raises the mechanical arms by an input angle
	 * @param angle The angle in which to rotate the arm
	 * @return void
	 */
	public void raiseArms(int angle){
		/*
		armMotor.rotate(300, false);
		armMotor.stop();
		clampMotor.lock(50);
		*/
		
	}
	
	
	/**
	 * Clamps the claws together
	 * @return void
	 */
	public void clamp(){
		//clampMotor.setSpeed(100);
		//clampMotor.backward();
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
