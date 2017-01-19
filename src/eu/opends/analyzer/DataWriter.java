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

package eu.opends.analyzer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URI;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import eu.opends.car.Car;
import eu.opends.tools.Util;
import eu.opends.traffic.TrafficCar;
import eu.opends.car.Transmission;

/**
 * 
 * That class is responsible for writing drive-data. At the moment it is a
 * ripped down version of similar classes used in CARS.
 * 
 * @author Saied
 * 
 */
public class DataWriter 
{
	private Calendar startTime = new GregorianCalendar();

	/**
	 * An array list for not having to write every row directly to file.
	 */
	private ArrayList<DataUnit> arrayDataList;
	private ArrayList<DataUnit> arrayDataList_t1;
	/*private ArrayList<DataUnit> arrayDataList_t2;
	private ArrayList<DataUnit> arrayDataList_t3;
	private ArrayList<DataUnit> arrayDataList_t4;
	private ArrayList<DataUnit> arrayDataList_t5;
	private ArrayList<DataUnit> arrayDataList_t6;*/
	private ArrayList<Float> rpm_list;
	private BufferedWriter out;
	private File outFile;
	private String newLine = System.getProperty("line.separator");
	private Date lastAnalyzerDataSave;
	private Car car;
	private TrafficCar tcar1;
	/*private TrafficCar tcar2;
	private TrafficCar tcar3;
	private TrafficCar tcar4;
	private TrafficCar tcar5;
	private TrafficCar tcar6;*/
	private File analyzerDataFile;
	private boolean dataWriterEnabled = false;
	private Date curDate;
	private String relativeDrivingTaskPath;
	private Transmission t;


	//public DataWriter(String outputFolder, Car car, TrafficCar tcar1, TrafficCar tcar2, TrafficCar tcar3, TrafficCar tcar4, TrafficCar tcar5, TrafficCar tcar6, String driverName, String absoluteDrivingTaskPath, int trackNumber)
	public DataWriter(String outputFolder, Car car, TrafficCar tcar1, String driverName, String absoluteDrivingTaskPath, int trackNumber)
	{
		this.car = car;
		this.tcar1 = tcar1;
		/*this.tcar2 = tcar2;
		this.tcar3 = tcar3;
		this.tcar4 = tcar4;
		this.tcar5 = tcar5;
		this.tcar6 = tcar6;*/
		this.relativeDrivingTaskPath = getRelativePath(absoluteDrivingTaskPath);
		t = car.getTransmission();
		
		Util.makeDirectory(outputFolder);

		if(trackNumber >= 0)
			analyzerDataFile = new File(outputFolder + "/carData_track" + trackNumber + ".txt");
		else
			analyzerDataFile = new File(outputFolder + "/carData.txt");

		
		if (analyzerDataFile.getAbsolutePath() == null) 
		{
			System.err.println("Parameter not accepted at method initWriter.");
			return;
		}
		
		outFile = new File(analyzerDataFile.getAbsolutePath());
		
		int i = 2;
		while(outFile.exists()) 
		{
			if(trackNumber >= 0)
				analyzerDataFile = new File(outputFolder + "/carData_track" + trackNumber + "(" + i + ").txt");
			else
				analyzerDataFile = new File(outputFolder + "/carData(" + i + ").txt");
			
			outFile = new File(analyzerDataFile.getAbsolutePath());
			i++;
		}
		
		
		try {
			out = new BufferedWriter(new FileWriter(outFile));
			out.write("Driving Task: " + relativeDrivingTaskPath + newLine);
			out.write("Date-Time: "
					+ new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss")
							.format(new Date()) + newLine);
			out.write("Driver: " + driverName + newLine);
			//added traffic car speed
			out.write("Used Format = Time (ms): CarPosition (x,y,z) : CarRotation (x,y,z,w) :"
					+ " CarSpeed (km/h) : CarSteering Wheel Position [-1,1] : Car Gas Pedal Position :"
					+ " Car Brake Pedal Position : Car Engine Running " 
					+ "TCar1Position (x,y,z) " + ": TCar1Rotation (x,y,z,w) :" + " TCar1Speed (km/h)"
					/*+ "TCar2Position (x,y,z) " + ": TCar2Rotation (x,y,z,w) :" + " TCar2Speed (km/h)"
					+ "TCar3Position (x,y,z) " + ": TCar3Rotation (x,y,z,w) :" + " TCar3Speed (km/h)"
					+ "TCar4Position (x,y,z) " + ": TCar4Rotation (x,y,z,w) :" + " TCar4Speed (km/h)"
					+ "TCar5Position (x,y,z) " + ": TCar5Rotation (x,y,z,w) :" + " TCar5Speed (km/h)"
					+ "TCar6Position (x,y,z) " + ": TCar6Rotation (x,y,z,w) :" + " TCar6Speed (km/h) :"*/
					+ "rpm" + newLine);

		} catch (IOException e) {
			e.printStackTrace();
		}
		arrayDataList = new ArrayList<DataUnit>();
		arrayDataList_t1 = new ArrayList<DataUnit>();
		/*arrayDataList_t2 = new ArrayList<DataUnit>();
		arrayDataList_t3 = new ArrayList<DataUnit>();
		arrayDataList_t4 = new ArrayList<DataUnit>();
		arrayDataList_t5 = new ArrayList<DataUnit>();
		arrayDataList_t6 = new ArrayList<DataUnit>();*/
		rpm_list = new ArrayList<Float>();
		lastAnalyzerDataSave = new Date();
	}
	
	
	private String getRelativePath(String absolutePath)
	{
		URI baseURI = new File("./").toURI();
		URI absoluteURI = new File(absolutePath).toURI();
		URI relativeURI = baseURI.relativize(absoluteURI);
		
		return relativeURI.getPath();
	}


	/**
	 * Save the car data at a frequency of 20Hz. That class should be called in
	 * the update-method <code>Simulator.java</code>.
	 */
	public void saveAnalyzerData() 
	{
		curDate = new Date(); //this method is slower than System.currentTimeMillis() may be by a few micro-secs

		if ((curDate.getTime() - lastAnalyzerDataSave.getTime()) >= 16)
		{
			write(
					curDate,
					Math.round(car.getPosition().x * 1000) / 1000.0f,
					Math.round(car.getPosition().y * 1000) / 1000.0f,
					Math.round(car.getPosition().z * 1000) / 1000.0f,
					Math.round(car.getRotation().getX() * 10000) / 10000.0f,
					Math.round(car.getRotation().getY() * 10000) / 10000.0f,
					Math.round(car.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(car.getRotation().getW() * 10000) / 10000.0f,
					car.getCurrentSpeedKmhRounded(), 
					Math.round(car.getSteeringWheelState() * 100000) / 100000.0f, 
					car.getAcceleratorPedalIntensity(), car.getBrakePedalIntensity(), 
				    car.isEngineOn(), 
				    Math.round(tcar1.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar1.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar1.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar1.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar1.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar1.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar1.getRotation().getW() * 10000) / 10000.0f,
					tcar1.getCurrentSpeedKmhRounded(), 
					Math.round(tcar1.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar1.getAcceleratorPedalIntensity(), tcar1.getBrakePedalIntensity(), 
					tcar1.isEngineOn(),
					/*Math.round(tcar2.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar2.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar2.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar2.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar2.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar2.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar2.getRotation().getW() * 10000) / 10000.0f,
					tcar2.getCurrentSpeedKmhRounded(), 
					Math.round(tcar2.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar2.getAcceleratorPedalIntensity(), tcar2.getBrakePedalIntensity(), 
					tcar2.isEngineOn(),
					Math.round(tcar3.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar3.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar3.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar3.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar3.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar3.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar3.getRotation().getW() * 10000) / 10000.0f,
					tcar3.getCurrentSpeedKmhRounded(), 
					Math.round(tcar3.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar3.getAcceleratorPedalIntensity(), tcar3.getBrakePedalIntensity(), 
					tcar3.isEngineOn(), 
					Math.round(tcar4.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar4.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar4.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar4.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar4.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar4.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar4.getRotation().getW() * 10000) / 10000.0f,
					tcar4.getCurrentSpeedKmhRounded(), 
					Math.round(tcar4.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar4.getAcceleratorPedalIntensity(), tcar4.getBrakePedalIntensity(), 
					tcar4.isEngineOn(),
					Math.round(tcar5.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar5.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar5.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar5.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar5.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar5.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar5.getRotation().getW() * 10000) / 10000.0f,
					tcar5.getCurrentSpeedKmhRounded(), 
					Math.round(tcar5.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar5.getAcceleratorPedalIntensity(), tcar5.getBrakePedalIntensity(), 
					tcar5.isEngineOn(), 
					Math.round(tcar6.getPosition().x * 1000) / 1000.0f,
					Math.round(tcar6.getPosition().y * 1000) / 1000.0f,
					Math.round(tcar6.getPosition().z * 1000) / 1000.0f,
					Math.round(tcar6.getRotation().getX() * 10000) / 10000.0f,
					Math.round(tcar6.getRotation().getY() * 10000) / 10000.0f,
					Math.round(tcar6.getRotation().getZ() * 10000) / 10000.0f,
					Math.round(tcar6.getRotation().getW() * 10000) / 10000.0f,
					tcar6.getCurrentSpeedKmhRounded(), 
					Math.round(tcar6.getSteeringWheelState() * 100000) / 100000.0f, 
					tcar6.getAcceleratorPedalIntensity(), tcar6.getBrakePedalIntensity(), 
					tcar6.isEngineOn(),*/ t.getRPM()
					);

			lastAnalyzerDataSave = curDate;
		}

	}

	
	// see eu.opends.analyzer.IAnalyzationDataWriter#write(float,
	//      float, float, float, java.util.Date, float, float, boolean, float)
	public void write(Date curDate, float x1, float y1, float z1, float xRot1,
			float yRot1, float zRot1, float wRot1, float linearSpeed1,
			float steeringWheelState1, float gasPedalState1, float brakePedalState1,
			boolean isEngineOn1, float x2, float y2, float z2, float xRot2,
			float yRot2, float zRot2, float wRot2, float linearSpeed2,
			float steeringWheelState2, float gasPedalState2, float brakePedalState2,
			boolean isEngineOn2,  /*float x3, float y3, float z3, float xRot3,
			float yRot3, float zRot3, float wRot3, float linearSpeed3,
			float steeringWheelState3, float gasPedalState3, float brakePedalState3,
			boolean isEngineOn3, float x4, float y4, float z4, float xRot4,
			float yRot4, float zRot4, float wRot4, float linearSpeed4,
			float steeringWheelState4, float gasPedalState4, float brakePedalState4,
			boolean isEngineOn4,float x5, float y5, float z5, float xRot5,
			float yRot5, float zRot5, float wRot5, float linearSpeed5,
			float steeringWheelState5, float gasPedalState5, float brakePedalState5,
			boolean isEngineOn5, float x6, float y6, float z6, float xRot6,
			float yRot6, float zRot6, float wRot6, float linearSpeed6,
			float steeringWheelState6, float gasPedalState6, float brakePedalState6,
			boolean isEngineOn6, float x7, float y7, float z7, float xRot7,
			float yRot7, float zRot7, float wRot7, float linearSpeed7,
			float steeringWheelState7, float gasPedalState7, float brakePedalState7,
			boolean isEngineOn7,*/ float rpmstate)
	/*public void write(Date curDate, float x1, float y1, float z1, float xRot1,
			float yRot1, float zRot1, float wRot1, float linearSpeed1,
			float steeringWheelState1, float gasPedalState1, float brakePedalState1,
			boolean isEngineOn1) */
	{
		//data from original steering car
		DataUnit row1 = new DataUnit(curDate, x1, y1, z1, xRot1, yRot1, zRot1, wRot1,
				linearSpeed1, steeringWheelState1, gasPedalState1, brakePedalState1,
				isEngineOn1);
		//data from one traffic car
		DataUnit row2 = new DataUnit(curDate, x2, y2, z2, xRot2, yRot2, zRot2, wRot2,
				linearSpeed2, steeringWheelState2, gasPedalState2, brakePedalState2,
				isEngineOn2);
		/*DataUnit row3 = new DataUnit(curDate, x3, y3, z3, xRot3, yRot3, zRot3, wRot3,
				linearSpeed3, steeringWheelState3, gasPedalState3, brakePedalState3,
				isEngineOn3);
		DataUnit row4 = new DataUnit(curDate, x4, y4, z4, xRot4, yRot4, zRot4, wRot4,
				linearSpeed4, steeringWheelState4, gasPedalState4, brakePedalState4,
				isEngineOn4);
		DataUnit row5 = new DataUnit(curDate, x5, y5, z5, xRot5, yRot5, zRot5, wRot5,
				linearSpeed5, steeringWheelState5, gasPedalState5, brakePedalState5,
				isEngineOn5);
		DataUnit row6 = new DataUnit(curDate, x6, y6, z6, xRot6, yRot6, zRot6, wRot6,
				linearSpeed6, steeringWheelState6, gasPedalState6, brakePedalState6,
				isEngineOn6);
		DataUnit row7 = new DataUnit(curDate, x7, y7, z7, xRot7, yRot7, zRot7, wRot7,
				linearSpeed7, steeringWheelState7, gasPedalState7, brakePedalState7,
				isEngineOn7);*/
		
		
		//this.write(row1, row2, row3, row4, row5, row6, row7,rpmstate);
		this.write(row1, row2,rpmstate);
		//this.write(row1);

	}
	

	/**
	 * Write data to the data pool. After 50 data sets, the pool is flushed to
	 * the file.
	 * 
	 * @param row1
	 * 			Datarow to write
	 */
	//public void write(DataUnit row1, DataUnit row2, DataUnit row3, DataUnit row4, DataUnit row5, DataUnit row6, DataUnit row7, float rpmstate)
	public void write(DataUnit row1, DataUnit row2, float rpmstate)
	//public void write(DataUnit row1)
	{
		arrayDataList.add(row1);
		arrayDataList_t1.add(row2);
		/*arrayDataList_t2.add(row3);
		arrayDataList_t3.add(row4);
		arrayDataList_t4.add(row5);
		arrayDataList_t5.add(row6);
		arrayDataList_t6.add(row7);*/
		rpm_list.add(rpmstate);
		if (arrayDataList.size() > 50)
			flush();
	}
	

	public void flush() 
	{
		try {
			StringBuffer sb = new StringBuffer();
			//int length = (arrayDataList.size() > arrayDataList_t2.size() ? arrayDataList_t2.size():arrayDataList.size());
			int length = arrayDataList.size();
			for (int i = 0; i < length; i++) {
				DataUnit r = arrayDataList.get(i);
				DataUnit s1 = arrayDataList_t1.get(i);
				/*DataUnit s2 = arrayDataList_t2.get(i);
				DataUnit s3 = arrayDataList_t3.get(i);
				DataUnit s4 = arrayDataList_t4.get(i);
				DataUnit s5 = arrayDataList_t5.get(i);
				DataUnit s6 = arrayDataList_t6.get(i);*/
				float newrpm = rpm_list.get(i);
				sb.append(r.getDate().getTime() + ":" + r.getXpos() + ":"
						+ r.getYpos() + ":" + r.getZpos() + ":" + r.getXrot()
						+ ":" + r.getYrot() + ":" + r.getZrot() + ":"
						+ r.getWrot() + ":" + r.getSpeed() + ":"
						+ r.getSteeringWheelPos() + ":" + r.getAcceleratorPedalPos() + ":"
						+ r.getBrakePedalPos() + ":" + r.isEngineOn() + "\t" 
						+ s1.getXpos() + ":" + s1.getYpos() + ":" + s1.getZpos() + ":" + s1.getXrot() + 
						":" + s1.getYrot() + ":" + s1.getZrot() + ":" + s1.getWrot() + ":" + s1.getSpeed() + "\t" + 
						/*s2.getXpos() + ":" + s2.getYpos() + ":" + s2.getZpos() + ":" + s2.getXrot() +
						":" + s2.getYrot() + ":" + s2.getZrot() + ":" + s2.getWrot() + ":" + s2.getSpeed() + "\t"
						+ s3.getXpos() + ":" + s3.getYpos() + ":" + s3.getZpos() + ":" + s3.getXrot() + 
						":" + s3.getYrot() + ":" + s3.getZrot() + ":" + s3.getWrot() + ":" + s3.getSpeed() + "\t"
						+ s4.getXpos() + ":" + s4.getYpos() + ":" + s4.getZpos() + ":" + s4.getXrot() + 
						":" + s4.getYrot() + ":" + s4.getZrot() + ":" + s4.getWrot() + ":" + s4.getSpeed() + "\t"
						+ s5.getXpos() + ":" + s5.getYpos() + ":" + s5.getZpos() + ":" + s5.getXrot() + 
						":" + s5.getYrot() + ":" + s5.getZrot() + ":" + s5.getWrot() + ":" + s5.getSpeed() + "\t"
						+ s6.getXpos() + ":" + s6.getYpos() + ":" + s6.getZpos() + ":" + s6.getXrot() + 
						":" + s6.getYrot() + ":" + s6.getZrot() + ":" + s6.getWrot() + ":" + s6.getSpeed() + "\t" +*/ newrpm +
						newLine);
			}
			out.write(sb.toString());
			arrayDataList.clear();
			arrayDataList_t1.clear();
			/*arrayDataList_t2.clear();
			arrayDataList_t3.clear();
			arrayDataList_t4.clear();
			arrayDataList_t6.clear();*/
			rpm_list.clear();
			
			out.flush();
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(0);
		}
	}

	public void quit() 
	{
		dataWriterEnabled = false;
		flush();
		try {
			if (out != null)
				out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	

	public boolean isDataWriterEnabled() 
	{
		return dataWriterEnabled;
	}

	
	public void setDataWriterEnabled(boolean dataWriterEnabled) 
	{
		this.dataWriterEnabled = dataWriterEnabled;
	}

	
	public void setStartTime() 
	{
		this.startTime = new GregorianCalendar();
	}
	
	
	public String getElapsedTime()
	{
		Calendar now = new GregorianCalendar();
		
		long milliseconds1 = startTime.getTimeInMillis();
	    long milliseconds2 = now.getTimeInMillis();
	    
	    long elapsedMilliseconds = milliseconds2 - milliseconds1;
	    
	    return "Time elapsed: " + new SimpleDateFormat("mm:ss.SSS").format(elapsedMilliseconds);
	}

}
