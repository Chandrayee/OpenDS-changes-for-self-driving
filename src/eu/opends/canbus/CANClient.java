/*
*  This file is part of OpenDS (Open Source Driving Simulator).
*  Copyright (C) 2015 Rafael Math
*
*  OpenDS is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  OpenDS is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
*/

package eu.opends.canbus;

import java.net.Socket;
import java.net.SocketException;

import com.jme3.math.Vector3f;
import com.jme3.math.Quaternion;

//import java.util.Calendar;
//import java.util.GregorianCalendar;
import java.io.*;

//import com.jme3.math.Vector3f;

import eu.opends.car.Car;
import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.environment.XMLParser;
import eu.opends.main.SimulationDefaults;
import eu.opends.main.Simulator;
import eu.opends.tools.PanelCenter;

/**
 * This class represents the connector to the CAN-Interface. Steering, gas, brake and 
 * control instructions from the real car will be forwarded to the simulator; heading,
 * geo coordinates and speed will be sent back to the CAN-Interface in order to display
 * the position and speed on a in-car display. Furthermore trigger collisions can be 
 * sent to the CAN-Interface.
 * 
 * @author Rafael Math
 */
public class CANClient extends Thread
{
	// angle the real car wheel must be rotated for full lock in simulator 
	private float maxSteeringAngle;	
	private Simulator sim;
	private Car car;
	private Car tcar1;
	//tcar2,tcar3,tcar4,tcar5,tcar6;
	//private int framerate;
	private boolean stoprequested;
	private boolean errorOccurred;
	private float steeringAngle;
	private boolean doSteering;
	private boolean firstValue=true;
	private boolean test=true;
	//private Calendar timeOfLastFire;
	private PrintWriter printWriter;
	private Socket socket;
	public static float carspeed;
	public static float gasvalue;
	
	
	/**
	 * Creates a new TCP connection with the CAN-Interface at the given IP and port
	 * 
	 * @param sim
	 * 			The simulator
	 */
	public CANClient(Simulator sim)
    {
		super();
		
		this.sim = sim;
		this.car = sim.getCar();
		this.tcar1=sim.tcar1;
		/*this.tcar2=sim.tcar2;
		this.tcar3=sim.tcar3;
		this.tcar4=sim.tcar4;
		this.tcar5=sim.tcar5;
		this.tcar6=sim.tcar6;*/
		stoprequested = false;
		errorOccurred = false;
		steeringAngle = 0.0f;
		doSteering = false;
		//timeOfLastFire = new GregorianCalendar();
		
		SettingsLoader settingsLoader = Simulator.getDrivingTask().getSettingsLoader();
		String ip = settingsLoader.getSetting(Setting.CANInterface_ip, SimulationDefaults.CANInterface_ip);
		int port = settingsLoader.getSetting(Setting.CANInterface_port, SimulationDefaults.CANInterface_port);
		//framerate = settingsLoader.getSetting(Setting.CANInterface_updateRate, SimulationDefaults.CANInterface_updateRate);
		maxSteeringAngle = settingsLoader.getSetting(Setting.CANInterface_maxSteeringAngle, SimulationDefaults.CANInterface_maxSteeringAngle); 

		
		try {

			
			// connect to Server
			socket = new Socket(ip,port);
			socket.setSoTimeout(100);

		} catch (Exception e) {
			e.printStackTrace();
			System.err.println("No TCP connection possible to CAN-Interface at " + ip + ":" + port);
			errorOccurred = true;
		}
    }
	
    
	/**
	 * Listens for incoming CAN instructions (as XML), such as gas, brake, steering angle, 
	 * reset and change view, which will be forwarded to the XML-parser
	 */
	@Override
	public void run() 
	{
		// when loop is left, connection will be closed
		// loop will be left when requested or error occurred
		while(!stoprequested && !errorOccurred)
		{
			try {

				// delete "NUL" at the end of each line
				String message1 = readMessage(socket).replace("\0", "");
				String message = message1.substring(0,message1.lastIndexOf(">")+1);
				String carposition = message1.substring(message1.lastIndexOf(">")+1);
				String[] newposition = carposition.split(",");
				System.out.println(message);
				//driver car positions and rotations
				float x_position = Float.parseFloat(newposition[0]);
				float z_position = Float.parseFloat(newposition[1]);
				float x_rotation = Float.parseFloat(newposition[2]);
				float y_rotation = Float.parseFloat(newposition[3]);
				float z_rotation = Float.parseFloat(newposition[4]);
				float w_rotation = Float.parseFloat(newposition[5]);
				//traffic car 1 positions and rotations
				float car1_x_position = Float.parseFloat(newposition[6]);
				float car1_z_position = Float.parseFloat(newposition[7]);
				float car1_x_rotation = Float.parseFloat(newposition[8]);
				float car1_y_rotation = Float.parseFloat(newposition[9]);
				float car1_z_rotation = Float.parseFloat(newposition[10]);
				float car1_w_rotation = Float.parseFloat(newposition[11]);
				//traffic car 3 positions and rotations
				/*float car3_x_position = Float.parseFloat(newposition[12]);
				float car3_z_position = Float.parseFloat(newposition[13]);
				float car3_x_rotation = Float.parseFloat(newposition[14]);
				float car3_y_rotation = Float.parseFloat(newposition[15]);
				float car3_z_rotation = Float.parseFloat(newposition[16]);
				float car3_w_rotation = Float.parseFloat(newposition[17]);
				//traffic car 4 positions and rotations
				float car4_x_position = Float.parseFloat(newposition[18]);
				float car4_z_position = Float.parseFloat(newposition[19]);
				float car4_x_rotation = Float.parseFloat(newposition[20]);
				float car4_y_rotation = Float.parseFloat(newposition[21]);
				float car4_z_rotation = Float.parseFloat(newposition[22]);
				float car4_w_rotation = Float.parseFloat(newposition[23]);
				//traffic car 5 positions and rotations
				float car5_x_position = Float.parseFloat(newposition[24]);
				float car5_z_position = Float.parseFloat(newposition[25]);
				float car5_x_rotation = Float.parseFloat(newposition[26]);
				float car5_y_rotation = Float.parseFloat(newposition[27]);
				float car5_z_rotation = Float.parseFloat(newposition[28]);
				float car5_w_rotation = Float.parseFloat(newposition[29]);
				//traffic car 2 positions and rotations
				float car2_x_position = Float.parseFloat(newposition[30]);
				float car2_z_position = Float.parseFloat(newposition[31]);
				float car2_x_rotation = Float.parseFloat(newposition[32]);
				float car2_y_rotation = Float.parseFloat(newposition[33]);
				float car2_z_rotation = Float.parseFloat(newposition[34]);
				float car2_w_rotation = Float.parseFloat(newposition[35]);*/
				
				//carspeed = Float.parseFloat(newposition[8]);
				//gasvalue = Float.parseFloat(newposition[10]);
				
				
				//System.out.println(x_position + "," + z_position + "," + car3_x_position + "," + car3_z_position);
				//double distance = Math.sqrt(Math.pow(x_position-car.getPosition().getX(), 2)+Math.pow(z_position-car.getPosition().getZ(), 2));
				//if(distance>=0.25){
					//float yval =-5.671f;
					//float yval = -4.75f;
					//float yval = -5.48f;
					float yval = -0.350f;
					
					//for(int i=0;i!=newposition.length;i++)
						//System.out.println(newposition[i]);
					Quaternion q = new Quaternion(x_rotation, -y_rotation, z_rotation, w_rotation);
					car.setPosition(x_position,yval,z_position);
					car.setRotation(q);
					//PanelCenter.setSpeedIndicatorRotation(.5f);
					
					Quaternion q1 = new Quaternion(car1_x_rotation,-car1_y_rotation,car1_z_rotation,car1_w_rotation);
					tcar1.setPosition(car1_x_position,yval,car1_z_position);
					tcar1.setRotation(q1);
					
					/*Quaternion q2 = new Quaternion(car2_x_rotation,-car2_y_rotation,car2_z_rotation,car2_w_rotation);
					tcar2.setPosition(car2_x_position,yval,car2_z_position);
					tcar2.setRotation(q2);
					
					Quaternion q3 = new Quaternion(car3_x_rotation,-car3_y_rotation,car3_z_rotation,car3_w_rotation);
					tcar3.setPosition(car3_x_position,yval,car3_z_position);
					tcar3.setRotation(q3);
					
					Quaternion q4 = new Quaternion(car4_x_rotation,-car4_y_rotation,car4_z_rotation,car4_w_rotation);
					tcar4.setPosition(car4_x_position,yval,car4_z_position);
					tcar4.setRotation(q4);*/
					
					/*Quaternion q5 = new Quaternion(car5_x_rotation,-car5_y_rotation,car5_z_rotation,car5_w_rotation);
					tcar5.setPosition(car5_x_position,yval,car5_z_position);
					tcar5.setRotation(q5);*/
					
					/*q.set(car6_x_rotation,-car6_y_rotation,car6_z_rotation,car6_w_rotation);
					tcar6.setPosition(car6_x_position,yval,car6_z_position);
					tcar6.setRotation(q);*/
			//	}
				
				// print XML instruction
				//System.out.println(message1);
				
				// parse and evaluate XML instruction
				XMLParser parser = new XMLParser("<CAN>" + message + "</CAN>");
				parser.evalCANInstruction(sim,this);
				
			} catch (SocketException e) {
				
				// will be thrown if e.g. server was shut down
				System.err.println("Socket error: Connection to CAN-Interface has to be closed");
				errorOccurred = true;
				
			} catch (Exception e) {
			}
			
			// set virtual car's steering angle to the given steering angle
			if(doSteering)
				updateSteeringAngle();
		}
		
		// close TCP connection to CAN-Interface if connected at all
		try {
			if ((socket != null) && (printWriter != null))
			{
				// wait for 10 ms
				try {Thread.sleep(100);} 
				catch (InterruptedException e){}

				printWriter.print("exit");
				printWriter.flush();
				
				// close socket connection
				printWriter.close();
				socket.close();
				
				System.out.println("Connection to CAN-Interface closed");
			}
		} catch (Exception ex) {
			System.err.println("Could not close connection to CAN-Interface");
		}
	}

	
	/**
	 * Sends car data to the CAN-Interface, such as heading, geo coordinates and speed.
	 */
	public synchronized void sendCarData()
	{
		/*
		// break, if no connection established
		if(socket == null || errorOccurred)
			return;
		
		// generate time stamp
		Calendar currentTime = new GregorianCalendar();
		
		// if enough time has passed by since last fire, the event will be forwarded
		if(forwardEvent(currentTime))
		{
			float speed = ((float) car.getCurrentSpeedKmhRounded());  // in kph
			float heading = car.getHeadingDegree();       // 0..360 degree
			Vector3f geoPosition = car.getGeoPosition();
			float latitude = geoPosition.getX();          // N-S position in model coordinates
			float longitude = geoPosition.getY();         // W-E position in model coordinates

			try { 	

			 	// send car data (speed, heading, latitude and longitude) to CAN-Interface and flush
				String positionString = "$SimCarState#" + speed + "#" + heading + "#" + latitude + "#" + longitude + "%";
				printWriter = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));
			 	printWriter.print(positionString);
			 	printWriter.flush();
			 				 	
			} catch (IOException e) {
				System.err.println("CANClient_sendCarData(): " + e.toString());
			}

		}
		*/
	}
	
	
	/**
	 * Sends trigger reports to the CAN-Interface if the simulated car has hit a trigger.
	 * 
	 * @param triggerID
	 * 			ID of the CAN-Trigger that will be sent to the CAN-Interface
	 */
	public synchronized void sendTriggerData(String triggerID)
	{
		/*
		// break, if no connection established
		if(socket == null || errorOccurred)
			return;

		try { 	

		 	// send trigger data to CAN-bus and flush
			String triggerString = "$SimCarTrigger#" + triggerID + "%";
			printWriter = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));
		 	printWriter.print(triggerString);
		 	printWriter.flush();
		 				 	
		} catch (IOException e) {
			System.err.println("CANClient_sendTriggerData(): " + e.toString());
		}
		*/
	}

	
	/**
	 * Sends the current deviation from the normative line to the CAN-Interface.
	 * 
	 * @param deviation
	 * 			Value representing the current deviation in meters from the 
	 * 			normative line.
	 */
	public synchronized void sendDeviationData(float deviation) 
	{
		/*
		// break, if no connection established
		if(socket == null || errorOccurred)
			return;

		try { 	

		 	// send deviation data to CAN-Interface and flush
			String positionString = "$SimDeviationState#" + deviation + "%";
			printWriter = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));
		 	printWriter.print(positionString);
		 	printWriter.flush();
		 				 	
		} catch (IOException e) {
			System.err.println("CANClient_sendDeviationData(): " + e.toString());
		}
		*/
	}
	
	
	/**
	 * Sets the target steering angle as read from the CAN-Interface in order 
	 * to synchronize with the current steering angle of the simulator. Sets 
	 * "doSteering" to true.
	 * 
	 * @param steeringAngle
	 * 			Steering angle as read from the real car
	 */
	public synchronized void setSteeringAngle(float steeringAngle) 
	{
		// set doSteering to true in order to perform steering instructions 
		// from the real car; otherwise the keyboard will suppress car steering
		this.doSteering = true;
		this.steeringAngle = steeringAngle;
	}
	
	
	/**
	 * Sets "doSteering" to false in order to suppress the steering of the real car.
	 * E.g. if the keyboard steering has higher priority
	 */
	public synchronized void suppressSteering() 
	{
		this.doSteering = false;
	}
	
	
	/**
	 * Requests the connection to close after the current loop
	 */
	public synchronized void requestStop() 
	{
		stoprequested = true;
	}
	
	
	/**
	 * Compares the current steering angle (in the simulator) with the given 
	 * steering angle (of the real car). The bigger the difference, the faster
	 * the steering angle of the simulator will be changed to the wanted value 
	 */
	private void updateSteeringAngle() 
	{
		try {
			
			// get target steering angle from real car
			// maximum angle will be matched to -1 or 1, respectively
			//float targetAngle = -Math.max(Math.min(steeringAngle/maxSteeringAngle,1),-1);
			if(steeringAngle!=0&&firstValue)
			{
				car.getCarControl().setLinearVelocity(new Vector3f(20.581f,0f,.27303f));
				firstValue=false;
			}
			float targetAngle = -steeringAngle;
			
			// print target (real car) steering angle
			System.out.println("target: " + targetAngle);
			
			// if target angle is close to straight ahead, steer straight ahead
			if((targetAngle >= -0.001f) && (targetAngle <= 0.001f))	
				targetAngle = 0;
			
			car.steer(targetAngle);
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		
	}
    
	
	/**
	 * Reads an incoming message from the socket connection.
	 * 
	 * @param socket
	 * 			Socket connection
	 * 
	 * @return
	 * 			Message string of up to 10,000 characters
	 * 
	 * @throws IOException
	 */
	private String readMessage(Socket socket) throws IOException 
	{
		BufferedReader bufferedReader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
		char[] buffer = new char[10000];
		int nrOfChars = bufferedReader.read(buffer, 0, buffer.length);
		
		return new String(buffer, 0, nrOfChars);
	}
	
	
	/**
	 * This method checks whether the incoming camera information should 
	 * be sent to the server at the current time complying with the given 
	 * frame rate
	 * 
	 * @param now
	 * 			The current time stamp
	 * 
	 * @return true if enough time has passed by since last fire, false otherwise
	 */
	/*
    private boolean forwardEvent(Calendar now)
    {
        // fire an event every x milliseconds
    	int fireInterval = 1000 / framerate;

        // subtract time of last event from current time to get time elapsed since last fire
        long elapsedMillisecs = timeDiff(now,timeOfLastFire);
        
        if (elapsedMillisecs >= fireInterval)
        {
            // update time of last fire
            timeOfLastFire.add(Calendar.MILLISECOND, fireInterval);

            // fire
            return true;
        }
        else
            // do not fire
            return false;
    }
    */
	
    
	/**
	 * This method computes the difference between two given time stamps
	 * 
	 * @param timestamp1
	 * 			First time stamp value to compare
	 * 
	 * @param timestamp2
	 * 			Second time stamp value to compare
	 * 
	 * @return difference between the given time stamps in milliseconds (long)
	 */
	/*
    private static long timeDiff(Calendar timestamp1, Calendar timestamp2)
    {
        return Math.abs(timestamp1.getTimeInMillis() - timestamp2.getTimeInMillis());
    }
	 */
   
}
