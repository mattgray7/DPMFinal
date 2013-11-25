package Master;

import bluetooth.BluetoothConnection;
import bluetooth.Transmission;
import lejos.nxt.*;
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
		
		controller.execute();
		
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
		localization = new Localization(odo, bottomUs, nav, csOdoRight);

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
		bts.establishConnection();
		
		// Start sensor threads
		lcdInfo.start();
		odo.start();
		nav.start();
		// Do localization
		//localization.doLocalization();
		//nav.travelTo(0, 0);
		//nav.turnTo(90, true, true);
		//localization.doLightLocalization();

		// Start main operation
		//odometryCorrection.start();
		
		/*odometryCorrection.setMode(Mode.WAITING);
		nav.turnTo(7.6, true, true);
		odometryCorrection.setMode(Mode.CORRECTING);*/
		
		//nav.travelTo(0.0, 90.0);
		//nav.travelTo(60.0, 0.0);

		
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
		nav.setGX0(greenZone[0]*30);
		nav.setGY0(greenZone[1]*30);
		nav.setGX1(greenZone[2]*30);
		nav.setGY1(greenZone[3]*30);
	}
}
