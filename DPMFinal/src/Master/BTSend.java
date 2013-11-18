package Master;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.bluetooth.RemoteDevice;

import bluetooth.BluetoothConnection;
import bluetooth.Transmission;
import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;


public class BTSend {

	private static BTConnection connection;
	
	
	
	/**
	 * Send a signal to the slave brick and wait for response
	 * @param signal The integer number of which command the slave should execute
	 * @throws IOException Send or receive signal failure
	 * @return void
	 */
	public void sendSignal(int signal) throws IOException{
		DataOutputStream output = connection.openDataOutputStream();
		try
		{
			output.writeInt(signal);
			output.flush();
			output.close();
			
			
		}
		catch(Exception ioe)
		{
			Sound.beep();
			LCD.drawString("Could not send signal", 0, 0, false);
		}
	}
	
	/**
	 * Sets up BlueTooth connection immediately
	 * @return True once the connection has been made
	 * @return False if connection exception detected
	 */
	public void establishConnection(){
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
			//return true;

		} catch(Exception ioe){
			LCD.clear();
			LCD.drawString("ERROR", 0, 0);
			LCD.drawString(ioe.getMessage(), 0, 2);
			LCD.refresh();
			Sound.buzz();
			//return false;
		}
		
	}
	

	

	
}
