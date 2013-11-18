package Master;

import bluetooth.BluetoothConnection;
import bluetooth.Transmission;
import lejos.nxt.*;
import Master.Odometer;


public class BTSendController {
	private static final double LEFT_WHEEL_RADIUS = 2.665;
	private static final double RIGHT_WHEEL_RADIUS = 2.675;
	private static final double WHEELBASE_WIDTH = 17.3;
	private static final int DEFAULT_PERIOD = 25;
	
	private static final NXTRegulatedMotor leftMotor = Motor.B;
	private static final NXTRegulatedMotor rightMotor = Motor.C;
	private static final NXTRegulatedMotor sensorMotor = Motor.A;
	private static final SensorPort US_PORT = SensorPort.S1;
	private static final SensorPort CS_PORT = SensorPort.S2;
	private static final SensorPort CS_ODO_CORRECTION_PORT = SensorPort.S3;
	
	public UltrasonicSensor bottomUs;
	public ColorSensor odoSensor;
	public ColorSensor colorSensor;
	public Odometer odo;
	public BTSend bts;
	public ObjectRecognition or;
	public Navigation nav;
	public OdometryCorrection oc;
	public LCDInfo lc;
	public Localization usl;
	
	/**
	 * Constructor
	 */
	public BTSendController(){
		bottomUs = new UltrasonicSensor(US_PORT);
		odoSensor= new ColorSensor(CS_PORT);
		colorSensor = new ColorSensor(CS_ODO_CORRECTION_PORT);
		
		odo = new Odometer(LEFT_WHEEL_RADIUS, RIGHT_WHEEL_RADIUS, WHEELBASE_WIDTH);
		bts = new BTSend();
		or = new ObjectRecognition(colorSensor);
		nav = new Navigation(odo, bts, bottomUs, colorSensor, or);
		oc = new OdometryCorrection(odo, odoSensor, nav);
		lc = new LCDInfo(odo);
		usl = new Localization(odo, bottomUs, nav, odoSensor);
	}
	
	public void execute(){
		int buttonChoice = Button.waitForAnyPress();
		
		colorSensor.setFloodlight(true);
		or.calibrateBlueBlock();
		
		getTransmission();
		LCDInfo lc = new LCDInfo(odo);
		bts.establishConnection();
		odo.start();
		
		usl.doLocalization();
		usl.doLightLocalization();

		//oc.start();
		nav.start();
	}
	
	/**
	 * Setup a BTSendController (the main bulk of the program) and start it.
	 * 
	 * @return void
	 */
	public static void main(String[] args){
		BTSendController controller = new BTSendController();
		
		controller.execute();
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
		
		int greenZone[] = t.greenZone;
		nav.setGX0(greenZone[0]*30);
		nav.setGY0(greenZone[1]*30);
		nav.setGX1(greenZone[2]*30);
		nav.setGY1(greenZone[3]*30);
	}
}
