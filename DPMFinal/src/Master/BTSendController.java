package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import bluetooth.BluetoothConnection;
import bluetooth.Transmission;
import lejos.nxt.*;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;
import Master.Odometer;


public class BTSendController {
	
	public static final double LW_RADIUS = 2.665;
	public static final double RW_RADIUS = 2.675;
	public static final double WHEEL_BASE = 17.3;
	
	public static UltrasonicSensor bottomUs = new UltrasonicSensor(SensorPort.S1);
	public static ColorSensor rightColorSensor= new ColorSensor(SensorPort.S2);
	public static ColorSensor colorSensor = new ColorSensor(SensorPort.S3);
	public static ColorSensor leftColorSensor = new ColorSensor(SensorPort.S4);
	//public static UltrasonicSensor topUs = new UltrasonicSensor(SensorPort.S4);
	
	
	public static final int DEFAULT_PERIOD = 25;
	public static Odometer odo = new Odometer(LW_RADIUS, RW_RADIUS, WHEEL_BASE);
	public static BTSend bts = new BTSend();
	
	public static ObjectRecognition or = new ObjectRecognition(colorSensor);
	public static Navigation nav = new Navigation(odo, bts, bottomUs, colorSensor, or);
	public static OdometryCorrection oc = new OdometryCorrection(odo, leftColorSensor, rightColorSensor, nav);


	public static Localization usl = new Localization(odo, bottomUs, nav, rightColorSensor);
	

	private NXTRegulatedMotor sensorMotor = Motor.A;
	private NXTRegulatedMotor leftMotor = Motor.B;
	private NXTRegulatedMotor rightMotor = Motor.C;
	
	
	/**
	 * Main control flow of system
	 * @return void
	 */
	public static void main(String[] args){
		//start the program
		int buttonChoice = Button.waitForAnyPress();
		
		//calibrate light sensor with blue block
		//colorSensor.setFloodlight(true);
		//or.calibrateBlueBlock();
		
		//get role, starting position, green and red zone coordinates
		//getTransmission();
		
		//start odometry display
		LCDInfo lc = new LCDInfo(odo);
		
		//connect to slave brick
		//bts.establishConnection();
		
		//localize
		odo.start();
		//usl.doLocalization();
		//usl.doLightLocalization();

		//start odometry correction once localization is complete
		oc.start();
		
		//start main navigation
		nav.start();

		
		
		
		//connection.close();
	}
	
	/**
	 * Receives packet containing robot role, starting location, and the location of the red and green zones
	 * @return void
	 */
	public static void getTransmission(){
		BluetoothConnection compConnection = new BluetoothConnection();
		Transmission t = compConnection.getTransmission();
		
		//set the green zone coordinates
		int greenZone[] = t.greenZone;
		nav.setGX0(greenZone[0]*30);
		nav.setGY0(greenZone[1]*30);
		nav.setGX1(greenZone[2]*30);
		nav.setGY1(greenZone[3]*30);
		
	}
	
	
}
