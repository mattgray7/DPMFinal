

package Slave;
import lejos.nxt.*;

public class Lift {
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
	
	/*public static void main(String[] args){
		lowerArms(400);
		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e) {}
		
		release();
		Sound.beep();
		try {Thread.sleep(1000);} catch (InterruptedException e) {}
		
		clamp();
		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e) {}
		
		raiseArms(400);
		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e) {}
		
		//lowerArms(230);
		lowerArms(170);
		Sound.beep();
		try {Thread.sleep(500);} catch (InterruptedException e) {}
		
		release();
		Sound.buzz();
		try {Thread.sleep(500);} catch (InterruptedException e) {}
		
	}*/
	
	
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
		armMotor.setSpeed(450);
		armMotor.forward();
		armMotor.rotate(angle, false);//maybe true?
		armMotor.stop();
		
		
	}
	
	
	/**
	 * Clamps the claws together
	 * @return void
	 */
	public static void clamp(){
		clampMotor.setSpeed(150);
		clampMotor.backward();
		clampMotor.rotate(-90, false);
		clampMotor.flt();

	}
	
	
	/**
	 * Opens the claws
	 * @return void
	 */
	public static void release(){
		clampMotor.setSpeed(100);
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
