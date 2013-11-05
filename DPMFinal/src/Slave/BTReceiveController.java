package Slave;

import lejos.nxt.ColorSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.*;

public class BTReceiveController {
	
	public static ColorSensor colorSens= new ColorSensor(SensorPort.S1);
	private NXTRegulatedMotor armMotor = Motor.A;
	private NXTRegulatedMotor clampMotor = Motor.B;
	private Lift lift = new Lift(armMotor, clampMotor);
	
	/**
	 * Constructor
	 */
	public BTReceiveController(){
		
	}
	
	/**
	 * Waits for connection to be sent from master brick
	 * @return void
	 */
	public void establishConnection(){
		
	}
	
	/**
	 * After connection is made, slave brick will wait for signal from master brick
	 * @return void
	 */
	public void waitForSignal(){
		
	}
	
	/**
	 * Slave brick is in control, will control execution from here
	 * @param signal The signal sent from the master brick indicating which operation the slave should perform
	 * @return void
	 */
	public void hasControl(int signal){
		
	}
	
	/**
	 * Sends a confirmation or failure signal back to the master brick indicating that the slave process has completed
	 * @param signal The confirmation or failure signal
	 * @return void
	 */
	public void replySignal(int signal){
		
	}
}
