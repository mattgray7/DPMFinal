package Slave;
import lejos.nxt.*;

/**
 * Class for operating the lifting arms.
 * 
 * @author Matt ?
 *
 */
public class Lift {
	private static final int LOWERING_SPEED = 200;
	private static final int RAISING_SPEED = 400;
	private static final int CLAMPING_SPEED = 150;
	
	private static NXTRegulatedMotor armMotor = Motor.A;
	private static NXTRegulatedMotor clampMotor = Motor.B;
	
	/**
	 * Constructor
	 * @param arm The arm motor of the lifting mechanism
	 * @param clamp The clamp motor that controls the claws
	 */
	public Lift(NXTRegulatedMotor arm, NXTRegulatedMotor clamp){
		//armMotor = Motor.A;//arm;
		//clampMotor = Motor.B;//clamp;
	}

	
	/**
	 * Lowers the mechanical arms by an input angle
	 * @param angle The angle in which to rotate the arm
	 * @return void
	 */
	public static void lowerArms(int angle){
		armMotor.setSpeed(LOWERING_SPEED);
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
		armMotor.setSpeed(RAISING_SPEED);
		armMotor.forward();
		armMotor.rotate(angle, false);//maybe true?
		armMotor.stop();
		
		
	}
	
	
	/**
	 * Clamps the claws together
	 * @return void
	 */
	public static void clamp(){
		clampMotor.setSpeed(CLAMPING_SPEED);
		clampMotor.backward();
		clampMotor.rotate(-100, false);
		clampMotor.lock(100);
	}
	
	
	/**
	 * Opens the claws
	 * @return void
	 */
	public static void release(){
		clampMotor.setSpeed(200);
		clampMotor.forward();
		clampMotor.rotate(95, false);
		clampMotor.stop();
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

}
