package Master;

import bluetooth.BluetoothConnection;
import bluetooth.Transmission;
import lejos.nxt.*;
import lejos.nxt.comm.RConsole;
import Master.Odometer;
import Master.OdometryCorrection.Mode;


public class BTSendController {
	public static final double LEFT_WHEEL_RADIUS = 2.665;
	public static final double RIGHT_WHEEL_RADIUS = 2.675;
	public static final double WHEELBASE_WIDTH = 17.3;
	private static final int DEFAULT_PERIOD = 25;
	
	private static final NXTRegulatedMotor leftMotor = Motor.B;
	private static final NXTRegulatedMotor rightMotor = Motor.C;
	private static final NXTRegulatedMotor sensorMotor = Motor.A;
	private static final SensorPort US_PORT = SensorPort.S1;
	private static final SensorPort CS_FRONT_PORT = SensorPort.S3;
	private static final SensorPort CS_ODO_LEFT_PORT = SensorPort.S4;
	private static final SensorPort CS_ODO_RIGHT_PORT = SensorPort.S2;
	
	private int corner = 1;
	
	private double startingX = 0.0;
	private double startingY = 0.0;
	
	public UltrasonicSensor bottomUs;
	public ColorSensor csOdoLeft;
	public ColorSensor csOdoRight;
	public ColorSensor csFront;
	public Odometer odo;
	public BTSend bts;
	public ObjectRecognition objectRecognition;
	public Navigation nav;
	public OdometryCorrection odometryCorrection;
	public LCDInfo lcdInfo;
	public Localization localization;
	public PathGenerator pathGenerator;
	
	
	/**
	 * Setup a BTSendController (the main bulk of the program) and start it.
	 * 
	 * @return void
	 */
	public static void main(String[] args){
		BTSendController controller = new BTSendController();
		
		// Un-comment to setup the RConsole connection
		controller.openRConsole();
		controller.execute();
		controller.closeRConsole();
		
		System.exit(0);
	}
	
	/**
	 * Constructor
	 */
	public BTSendController(){
		bottomUs = new UltrasonicSensor(US_PORT);
		csOdoLeft= new ColorSensor(CS_ODO_LEFT_PORT);
		csOdoRight= new ColorSensor(CS_ODO_RIGHT_PORT);
		csFront = new ColorSensor(CS_FRONT_PORT);
		
		odo = new Odometer(LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS, WHEELBASE_WIDTH);
		bts = new BTSend();
		objectRecognition = new ObjectRecognition(csFront);
		//pathGenerator = new PathGenerator(odo);
		pathGenerator = new PathGenerator(odo);
		nav = new Navigation(odo, bts, bottomUs, csFront, objectRecognition, pathGenerator);
		odometryCorrection = new OdometryCorrection(nav, odo, csOdoLeft, csOdoRight);
		lcdInfo = new LCDInfo(odo);
		localization = new Localization(odo, bottomUs, nav, csOdoLeft);

	}
	
	public void execute(){
		printWelcomeMessage();
		int buttonChoice = Button.waitForAnyPress();
		
		// Calibrate blue block
		csFront.setFloodlight(true);
		objectRecognition.calibrateBlueBlock();
		csFront.setFloodlight(false);
		
		// Set up bluetooth connections
		//getTransmission();;

		
		// Start sensor threads
		lcdInfo.start();
		odo.start();


		
		// Do localization
		localization.doLocalization();
		nav.travelTo(0.0, 0.0, true);
		localization.doLightLocalization();
		nav.turnTo(90, true, true);
		odometryCorrection.start();
		
		if(corner == 2){
			odo.setX(300);
			odo.setY(0);
			odo.setTheta(180.0);
		}else if (corner == 3){
			odo.setX(300);
			odo.setY(300);
			odo.setTheta(270.0);
		}else if (corner == 4){
			odo.setX(0);
			odo.setY(300);
			odo.setTheta(0.0);
		}
		
		bts.establishConnection();
		nav.start();
		
		Button.waitForAnyPress();
		
		// Cleanup
		csFront.setFloodlight(false);
		csOdoLeft.setFloodlight(false);
		csOdoRight.setFloodlight(false);
	}
	
	/**
	 * Message that says what the program is, and to please press
	 * a button to start.
	 */
	private void printWelcomeMessage(){
		LCD.clear();
		LCD.drawString("Welcome!", 0, 0);
		LCD.drawString("DPM project", 0, 2);
		LCD.drawString("Press button", 0, 3);
		LCD.drawString("to start.", 0, 4);
	}
	
	/**
	 * Get the initial BT transmission fromt a pc.
	 * 
	 * This will give the robot all the starting information: green zone,
	 * red zone, starting corner, etc.
	 */
	public void getTransmission(){
		BluetoothConnection compConnection = new BluetoothConnection();
		Transmission t = compConnection.getTransmission();
		
		//set the green zone coordinates
		int greenZone[] = t.greenZone;
		int redZone[] = t.greenZone;
		int role = t.role.getId();
		startingX = t.startingCorner.getX();
		startingY = t.startingCorner.getY();
		
		if(role == 1){
			//builder
			nav.setTransmission(greenZone, redZone, role, startingX, startingY);
		}else if (role == 2){
			//collector
			nav.setTransmission(redZone, greenZone, role, startingX, startingY);
		}
	}
	
	/**
	 * Setup an RConsole connection with a PC.
	 * <p>
	 * Nick thinks this should only be used for debugging, and probably not
	 * alongside another bluetooth connection (ex. with the slave brick).
	 * 
	 */
	public void openRConsole(){
		LCD.clear();
		LCD.drawString("< Left | Right >", 0, 0);
		LCD.drawString("       |        ", 0, 1);
		LCD.drawString(" BT    | No BT  ", 0, 2);
		int buttonChoice = Button.waitForAnyPress();
		if(buttonChoice == Button.ID_LEFT){
			// Try to open a connection to a pc
			RConsole.openBluetooth(10000);	// Wait for connection (max 10 sec)
		}
		else{
			// Don't connect to a pc
		}
		LCD.clear();
	}
	
	/**
	 * Close the RConsole connection
	 * 
	 */
	public void closeRConsole(){
		if(RConsole.isOpen()){
			RConsole.close();
		}
	}
}
