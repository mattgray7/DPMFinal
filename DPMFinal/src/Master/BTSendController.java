package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;


public class BTSendController {
	
	public static UltrasonicSensor topUs = new UltrasonicSensor(SensorPort.S1);
	public static ColorSensor leftLightSensor= new ColorSensor(SensorPort.S2);
	public static ColorSensor rightLightSensor = new ColorSensor(SensorPort.S3);
	public static UltrasonicSensor bottomUs = new UltrasonicSensor(SensorPort.S4);
	
	public static final int DEFAULT_PERIOD = 25;
	public static Odometer odo = new Odometer(DEFAULT_PERIOD, true);
	public static OdometryCorrection oc = new OdometryCorrection(odo, leftLightSensor, rightLightSensor);
	public static Navigation nav = new Navigation(odo, topUs, bottomUs);
	public static LCDInfo lc = new LCDInfo(odo);
	public static Localization usl = new Localization(odo, topUs);
	
	private static BTConnection connection;
	
	private NXTRegulatedMotor leftMotor;	//=to Motor.A
	private NXTRegulatedMotor rightMotor;
	private NXTRegulatedMotor sensorMotor;	//if we need a sensor motor
	

	//TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	
	
	/**
	 * Main control flow of system
	 * @return void
	 */
	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		nav.run();
		
		
		
		//connection.close();
	}
	
	
	/**
	 * Sets up BlueTooth connection immediately
	 * @return True once the connection has been made
	 * @return False if connection exception detected
	 */
	public Boolean establishConnection(){
		String name = "slave2";		//friendly name of other brick
		
		try{
			//make the connection
			RemoteDevice receiver = Bluetooth.getKnownDevice(name);
			connection = Bluetooth.connect(receiver);
			
			//wont continue until reciever is connected
			while(receiver == null){
				receiver = Bluetooth.getKnownDevice(name);
			}

			
			LCD.drawString("connected.", 0, 1);
			return true;

		} catch(Exception ioe){
			LCD.clear();
			LCD.drawString("ERROR", 0, 0);
			LCD.drawString(ioe.getMessage(), 0, 2);
			LCD.refresh();
			return false;
		}
		
	}
	
	
	/**
	 * Send a signal to the slave brick and wait for response
	 * @param signal The integer number of which command the slave should execute
	 * @throws IOException Send or receive signal failure
	 * @return void
	 */
	public void sendSignal(int signal) throws IOException{
		DataOutputStream output = connection.openDataOutputStream();
		DataInputStream input = connection.openDataInputStream();
		try
		{
			output.writeInt(signal);
			output.flush();
			int reply = input.readInt();	//blocking, will wait for reply signal (hopefully)
			input.close();
			output.close();
			
			
		}
		catch(Exception ioe)
		{
			LCD.drawString("Could not send signal", 0, 0, false);
		}
		output.close();
	}
	
	
	//called when the slave brick is operating, will pause execution until "finished" signal received
	/*public void slaveFlow(){
		DataInputStream input = connection.openDataInputStream();
		int reply = input.readInt();
		
	}*/
	
	
}
