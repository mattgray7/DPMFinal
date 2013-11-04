package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;


public class BTSendController {
	
	public static final int DEFAULT_PERIOD = 25;
	public static Odometer odo = new Odometer(DEFAULT_PERIOD, true);
	public static OdometryCorrection oc = new OdometryCorrection(odo);
	public static Navigation nav = new Navigation(odo);
	public static LCDInfo lc = new LCDInfo(odo);
	public static Localization usl = new Localization(odo);
	
	private static BTConnection connection;
	
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private NXTRegulatedMotor sensorMotor;	//if we need a sensor motor
	
	UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
	ColorSensor leftSensor= new ColorSensor(SensorPort.S2);
	ColorSensor rightSensor = new ColorSensor(SensorPort.S3);
	//TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	
	
	//main control flow (replaced masterFlow in UML diagram)
	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		nav.travelTo(0.0,0.0,false);
		
		
		
		connection.close();
	}
	
	
	//establish initial connection, returns true once connection established - NEED TO BE BOOLEAN??
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
	
	
	//sends signal to slave brick
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
