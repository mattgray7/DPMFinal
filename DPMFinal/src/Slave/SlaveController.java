package Slave;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

public class SlaveController {
	
	public static ColorSensor colorSens= new ColorSensor(SensorPort.S1);
	private static NXTRegulatedMotor armMotor = Motor.A;
	private static NXTRegulatedMotor clampMotor = Motor.B;
	private static Lift lift = new Lift(armMotor, clampMotor);
	private static BTConnection connection;
	
	private static int testCount = 0;
	
	/**
	 * Constructor
	 */
	public SlaveController(){
		
	}
	
	/**
	 * Waits for connection to be sent from master brick
	 * @return void
	 */
	public static void main(String[] args){
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
	public static void waitForSignal(){
		LCD.drawString(" " + testCount + " signals sent", 0, 4, false);
		DataInputStream input = connection.openDataInputStream();
		int command = 0;
		try {command = input.readInt();} catch (IOException e) {}	//blocking
		
		//test command
		if(command == 1){
			testCount++;
			Sound.beep();
			try {Thread.sleep(500);} catch (InterruptedException e) {}
			Sound.beep();
			lift.lowerArms(450);
			lift.release();
			//replySignal(1);
			Sound.beep();
			try {input.close();} catch (IOException e) {}
			waitForSignal();
		}
		
		if(command == 2){
			testCount++;
			Sound.beep();
			lift.clamp();
			lift.raiseArms(400);
			//replySignal(1);
			Sound.beep();
			try {input.close();} catch (IOException e) {}
			waitForSignal();
		}
		

		try {input.close();} catch (IOException e) {}
		connection.close();
		
		
	}
	
	/**
	 * Slave brick is in control, will control execution from here
	 * @param signal The signal sent from the master brick indicating which operation the slave should perform
	 * @return void
	 */
	public static void hasControl(int signal){
		
	}
	
	/**
	 * Sends a confirmation or failure signal back to the master brick indicating that the slave process has completed
	 * @param signal The confirmation or failure signal
	 * @return void
	 */
	public static void replySignal(int signal){
		DataOutputStream output = connection.openDataOutputStream();

		try
		{
			output.writeInt(signal);
			output.flush();
			output.close();
		}
		catch(Exception ioe)
		{
			Sound.beep();
			LCD.drawString("Could not send signal", 0, 0, false);
		}

	}
}
