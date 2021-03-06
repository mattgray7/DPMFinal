package Slave;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;


/**
 * Class to operate the slave brick.
 * <p>
 * Establishes a bluetooth connection with the master and then performs
 * the incoming tasks.
 * 
 * @author Matt Gray
 * @author Julien Liberta
 */
public class SlaveController {
	private static final NXTRegulatedMotor armMotor = Motor.A;
	private static final NXTRegulatedMotor clampMotor = Motor.B;
	private Lift lift;
	private BTConnection connection;
	
	private static final int MAX = 340;
	private static final int ONE_BLOCK_BELOW = 230;
	private static final int TWO_BLOCKS_BELOW = 140;
	
	
	/**
	 * Constructor
	 */
	public SlaveController(){
		connection = new BTConnection(0);
		lift = new Lift(armMotor, clampMotor);
	}
	
	/**
	 * Main execution of the slave brick.
	 * <p>
	 * Basically waits for signals from the master brick and executes the tasks.
	 * 
	 */
	public void execute(){
		LCD.clear();
		LCD.drawString("Receiver wait...", 0, 0);
		LCD.refresh();

		try
		{
			connection = Bluetooth.waitForConnection();
			if (connection == null){
				throw new IOException("Connect fail");
			}
			/*DataOutputStream output = connection.openDataOutputStream();
			
			output.writeInt(1);
			output.flush();
			output.close();*/
			
			waitForSignal();

		}
		catch(Exception ioe){
			/* Do nothing */
		}
	}
	
	/**
	 * Waits for connection to be sent from master brick.
	 * 
	 */
	public static void main(String[] args){
		SlaveController slaveController = new SlaveController();
		slaveController.execute();
	}
	
	/**
	 * After connection is made, slave brick will wait for signal from master brick.
	 * 
	 */
	public void waitForSignal(){
		LCD.drawString("WAITING", 0, 4, false);
		DataInputStream input = connection.openDataInputStream();
		int command = 0;
		try {command = input.readInt();} catch (IOException e) {Sound.buzz();}	//blocking
		
		//lower arms all the way
		if(command == 1){
			lift.lowerArms(MAX);
			lift.release();
			Sound.beep();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//clamp and raise arms all the way
		if(command == 2){
			Sound.beep();
			lift.clamp(); 
			lift.raiseArms(MAX);
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//clamp only
		if(command == 3){
			lift.clamp();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//raise arms
		if(command == 4){
			lift.raiseArms(MAX);
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		if(command == 5){
			lift.release();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//lower to height above 1 block tower
		if(command == 10){
			lift.lowerArms(ONE_BLOCK_BELOW);
			lift.release();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		if(command == -10){
			lift.clamp();
			lift.raiseArms(ONE_BLOCK_BELOW);
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//lift to height above 2 block tower
		if(command == 11){
			lift.lowerArms(TWO_BLOCKS_BELOW);
			lift.release();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//lower claw to floor and close to push the tower further into the green zone
		if(command == -11){
			lift.lowerArms(TWO_BLOCKS_BELOW);
			lift.clamp();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		//if only building one tower, does not need to lower claw and push, just lift back to top
		if(command == 12){
			lift.clamp();
			lift.raiseArms(TWO_BLOCKS_BELOW);
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		connection.close();
	}
}
