package Slave;

public class BTReceiveController {

	//constructor
	public BTReceiveController(){
		
	}
	
	//waits for initial connection
	public void establishConnection(){
		
	}
	
	//after initial connection and while master has control, slave just waits for a command
	public void waitForSignal(){
		
	}
	
	//called once a signal has been sent to slave, this method is the controller for the slave brick
	public void hasControl(){
		
	}
	
	//sends a confirmation signal to master to either pass control, or request a movement??
	public void replySignal(int signal){
		
	}
}
