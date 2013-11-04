package Master;

import lejos.nxt.ColorSensor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.*;


public class BTSendController {
	
	public static final int DEFAULT_PERIOD = 25;
	public static Odometer odo = new Odometer(DEFAULT_PERIOD, true);
	public static OdometryCorrection oc = new OdometryCorrection(odo);
	public static Navigation nav = new Navigation(odo);
	public static LCDInfo lc = new LCDInfo(odo);
	public static Localization usl = new Localization(odo);
	
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
		
		
	}
	
	
	//establish initial connection, returns true once connection established - NEED TO BE BOOLEAN??
	public Boolean establishConnection(){
		
		return false;
	}
	
	
	//sends signal to slave brick
	public void sendSignal(int signal){
		
	}
	
	
	//called when the slave brick is operating, will pause execution until "finished" signal received
	public void slaveFlow(){
		
	}
	
	
}
