package Slave;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

public class BTReceiveController {
	
	public static ColorSensor colorSens= new ColorSensor(SensorPort.S1);
	private NXTRegulatedMotor armMotor = Motor.A;
	private NXTRegulatedMotor clampMotor = Motor.B;
	private Lift lift = new Lift(armMotor, clampMotor);
	BTConnection connection;
	
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
		LCD.clear();
		LCD.drawString("Receiver wait...", 0, 0);
		LCD.refresh();

		try
		{
			connection = Bluetooth.waitForConnection();
			if (connection == null)
				throw new IOException("Connect fail");
			DataOutputStream output = connection.openDataOutputStream();
			
			output.writeInt(1);
			output.flush();
			output.close();
			
			waitForSignal();

		}
		catch(Exception ioe)
		{
		}
	
		
	}
	
	/**
	 * After connection is made, slave brick will wait for signal from master brick
	 * @return void
	 */
	public void waitForSignal(){
		LCD.drawString("WAIT", 0, 4, false);
		DataInputStream input = connection.openDataInputStream();
		int command = 0;
		try {
			command = input.readInt();
		} catch (IOException e) {}	//blocking
		
		//test command
		if(command == 2){
			Sound.beep();
			try {Thread.sleep(1000); } catch (InterruptedException e) {}
		}
		
		try {input.close();} catch (IOException e) {}
		connection.close();
		
		
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
