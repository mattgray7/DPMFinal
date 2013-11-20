package Slave;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

public class SlaveController {
	private static final NXTRegulatedMotor armMotor = Motor.A;
	private static final NXTRegulatedMotor clampMotor = Motor.B;
	private Lift lift;
	private BTConnection connection;
	
	
	/**
	 * Constructor
	 */
	public SlaveController(){
		connection = new BTConnection(0);
		lift = new Lift(armMotor, clampMotor);
	}
	
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
	 * Waits for connection to be sent from master brick
	 * @return void
	 */
	public static void main(String[] args){
		SlaveController slaveController = new SlaveController();
		slaveController.execute();
	}
	
	/**
	 * After connection is made, slave brick will wait for signal from master brick
	 * @return void
	 */
	public void waitForSignal(){
		LCD.drawString("WAITING", 0, 4, false);
		DataInputStream input = connection.openDataInputStream();
		int command = 0;
		try {command = input.readInt();} catch (IOException e) {Sound.buzz();}	//blocking
		
		//test command
		if(command == 1){
			lift.lowerArms(450);
			lift.release();
			Sound.beep();
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		
		if(command == 2){
			Sound.beep();
			lift.clamp();
			lift.raiseArms(400);
			try {input.close();} catch (IOException e) {Sound.buzz();}
			waitForSignal();
		}
		connection.close();
	}
}
