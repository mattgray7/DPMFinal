

package Slave;
import lejos.nxt.*;

public class Lift {
	private static NXTRegulatedMotor armMotor = Motor.A;
	private static NXTRegulatedMotor clampMotor = Motor.B;

	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		//lowerArms();
		//clamp();
		//raiseArms();
		
		try { Thread.sleep(2000); }catch (InterruptedException e) {}
		//release();
		//lowerArms();
		
	}
	
	
	//constructor
	public Lift(){
		
	}
	
	
	//lowers the mechanical arms
	public void lowerArms(){
		
		armMotor.setSpeed(150);
		armMotor.backward();
		
		
		Sound.beep();
		try { Thread.sleep(3000); }catch (InterruptedException e) {}
		Sound.beep();
	}
	
	//raise the mechanical arms
	public void raiseArms(){
		Sound.buzz();
		//armMotor.setSpeed(100);
		//armMotor.forward();
		//armMotor.rotate(300, false);
		armMotor.rotate(300, false);
		armMotor.stop();
		clampMotor.lock(50);
		Sound.beep();
		
		//Sound.beep();
		//try { Thread.sleep(4000); }catch (InterruptedException e) {}
		//Sound.beep();
		
	}
	
	
	//clamp the object - may not be necessary with current mechanical design
	public void clamp(){
		clampMotor.setSpeed(100);
		clampMotor.backward();
		
		try { Thread.sleep(1500); }catch (InterruptedException e) {}
		clampMotor.stop();
	}
	
	
	//release the clamps - again, not necessary with current design
	public void release(){
		clampMotor.setSpeed(100);
		clampMotor.forward();
		
		try { Thread.sleep(200); }catch (InterruptedException e) {}
		clampMotor.stop();
	}
	
	
	//If we connect an ultrasonic sensor to the arm, will return current distance to tower
	public int getTowerHeight(){
		return 0;
	}
	
	
	//will use raise and lower to position arm height, rotating the robot will knock a tower over
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
