package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;


public class BTSendController {
	
	public static UltrasonicSensor bottomUs = new UltrasonicSensor(SensorPort.S1);
	public static ColorSensor odometerSensor= new ColorSensor(SensorPort.S2);
	public static ColorSensor objectSensor = new ColorSensor(SensorPort.S3);
	public static UltrasonicSensor topUs = new UltrasonicSensor(SensorPort.S4);
	
	
	public static final int DEFAULT_PERIOD = 25;
	//public static Odometer odo = new Odometer(DEFAULT_PERIOD, true);
	public static Odometer odo = new Odometer(2.67, 2.668, 17.4);
	public static BTSend bts = new BTSend();
	
	public static Navigation nav = new Navigation(odo, bts, topUs, bottomUs, objectSensor);
	public static OdometryCorrection oc = new OdometryCorrection(odo, odometerSensor, nav);

	public static LCDInfo lc = new LCDInfo(odo);
	public static Localization usl = new Localization(odo, bottomUs);
	

	
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	private NXTRegulatedMotor sensorMotor;	//if we need a sensor motor
	

	//TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	
	
	/**
	 * Main control flow of system
	 * @return void
	 */
	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		//usl.doLocalization();
		//nav.turnTo(90.0, true);
		bts.establishConnection();

		//oc.start();
		odo.start();
		nav.start();

		
		
		
		//connection.close();
	}
	
	

	
	

	
	
	//called when the slave brick is operating, will pause execution until "finished" signal received
	/*public void slaveFlow(){
		DataInputStream input = connection.openDataInputStream();
		int reply = input.readInt();
		
	}*/
	
	
}
