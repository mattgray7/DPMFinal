package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import Master.Odometer;


public class BTSendController {
	
	public static final double LW_RADIUS = 2.675;
	public static final double RW_RADIUS = 2.685;
	public static final double WHEEL_BASE = 17.3;
	
	public static UltrasonicSensor bottomUs = new UltrasonicSensor(SensorPort.S1);
	public static ColorSensor odoSensor= new ColorSensor(SensorPort.S2);
	public static ColorSensor colorSensor = new ColorSensor(SensorPort.S3);
	//public static UltrasonicSensor topUs = new UltrasonicSensor(SensorPort.S4);
	
	
	public static final int DEFAULT_PERIOD = 25;
	//public static Odometer odo = new Odometer(DEFAULT_PERIOD, true);
	public static Odometer odo = new Odometer(LW_RADIUS, RW_RADIUS, WHEEL_BASE);
	public static BTSend bts = new BTSend();
	
	public static ObjectRecognition or = new ObjectRecognition(colorSensor);
	public static Navigation nav = new Navigation(odo, bts, bottomUs, colorSensor, or);
	public static OdometryCorrection oc = new OdometryCorrection(odo, odoSensor, nav);

	public static LCDInfo lc = new LCDInfo(odo);
	public static Localization usl = new Localization(odo, bottomUs, nav, odoSensor);
	

	
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	private NXTRegulatedMotor sensorMotor = Motor.A;	//if we need a sensor motor
	

	//TouchSensor touchSensor = new TouchSensor(SensorPort.S4);
	
	
	/**
	 * Main control flow of system
	 * @return void
	 */
	public static void main(String[] args){
		int buttonChoice = Button.waitForAnyPress();
		or.calibrateBlueBlock();
		bts.establishConnection();
		odo.start();
		//usl.doLocalization();
		//usl.doLightLocalization();


		//oc.start();

		nav.start();

		
		
		
		//connection.close();
	}
	
	

	
	//testcommit

	
	
	//called when the slave brick is operating, will pause execution until "finished" signal received
	/*public void slaveFlow(){
		DataInputStream input = connection.openDataInputStream();
		int reply = input.readInt();
		
	}*/
	
	
}
