import java.net.*;
import java.io.*;
import java.util.*;

public class tcpserver extends Thread
{
   public static int count = 0;
   public static ArrayList<Float> steeringAngles = new ArrayList<Float>();
   public static ArrayList<Float> accelerationList = new ArrayList<Float>();
   public static ArrayList<Float> brakeList = new ArrayList<Float>();	
   public static ArrayList<Long> timeDiff = new ArrayList<Long>();
   
   public static ArrayList<Float> xpos = new ArrayList<Float>();
   public static ArrayList<Float> zpos = new ArrayList<Float>();
   public static ArrayList<Float> xrot = new ArrayList<Float>();
   public static ArrayList<Float> yrot = new ArrayList<Float>();
   public static ArrayList<Float> zrot = new ArrayList<Float>();
   public static ArrayList<Float> wrot = new ArrayList<Float>();
   
   public static ArrayList<Float> xpos1 = new ArrayList<Float>();
   public static ArrayList<Float> zpos1 = new ArrayList<Float>();
   public static ArrayList<Float> xrot1 = new ArrayList<Float>();
   public static ArrayList<Float> yrot1 = new ArrayList<Float>();
   public static ArrayList<Float> zrot1 = new ArrayList<Float>();
   public static ArrayList<Float> wrot1 = new ArrayList<Float>();
    	

   private ServerSocket serversocket;
   
   public tcpserver(int port) throws IOException
   {
	serversocket = new ServerSocket(port);
	serversocket.setSoTimeout(100000);
   }

   public static void getData() throws IOException
   {
	 //Reading data from file
	FileInputStream fstream = new FileInputStream("carData_track1.txt");
	BufferedReader br = new BufferedReader(new InputStreamReader(fstream));
	long prevtime = 0;
	for(String line; (line = br.readLine()) != null; ) 
	{
		String[] fields = line.split(":");
		//length of fields is 13
		long time= Long.parseLong(fields[0]);
		long diff = time - prevtime;
		long diffinterp = diff;
		prevtime = time;
		xpos.add(Float.parseFloat(fields[1]));
		zpos.add(Float.parseFloat(fields[3]));
		xrot.add(Float.parseFloat(fields[4]));
		yrot.add(Float.parseFloat(fields[5]));
		zrot.add(Float.parseFloat(fields[6]));
		wrot.add(Float.parseFloat(fields[7]));
		
		float steering = Float.parseFloat(fields[9]);
		float gas = Float.parseFloat(fields[10]);
		float brake = Float.parseFloat(fields[11]);
		
		String[] lineend = fields[12].split("\t");
		xpos1.add(Float.parseFloat(lineend[1]));
		zpos1.add(Float.parseFloat(fields[14]));
		xrot1.add(Float.parseFloat(fields[15]));
		yrot1.add(Float.parseFloat(fields[16]));
		zrot1.add(Float.parseFloat(fields[17]));
		wrot1.add(Float.parseFloat(fields[18]));
		
		//commenting this part out
		/*if ((diff > 18) && (count > 1)) 
		{
			steeringAngles.add((steeringAngles.get(count-1) + steering)/2);
			accelerationList.add((accelerationList.get(count-1) + gas)/2);
			brakeList.add((brakeList.get(count-1) + brake)/2);
			timeDiff.add(new Long(17)); 
			diffinterp = diff - 17; 
			
		}*/
		steeringAngles.add(steering);
		accelerationList.add(gas);
		brakeList.add(brake);
		timeDiff.add(diffinterp);
		System.out.println(timeDiff.get(count));
		count++;
    }
	System.out.println(count);
	System.out.println(brakeList.size());
	br.close();
   }
   
   public void run()
   {
	
	while(true)
	{
	   try
           {
				System.out.println("Waiting for client on port " + 
                serversocket.getLocalPort() + "....");
				Socket server = serversocket.accept();
				System.out.println("Just connected to " + 
                server.getRemoteSocketAddress());
				long cumtimedifference = 0;

				//generate message every 50 milliseconds
				
				for(int i = 0; i < (count - 1); i++ )
				{
					StringBuilder message = new StringBuilder();
	
					message.append("<message>");
					message.append("\n	<action name=\"steering\">" + steeringAngles.get(i) + "</action>");
					message.append("\n	<action name=\"acceleration\">" + accelerationList.get(i) + "</action>");
					message.append("\n 	<action name=\"brake\">" + brakeList.get(i) + "</action>");
					message.append("\n</message>");
					message.append(xpos.get(i) + "," + zpos.get(i) + "," + xrot.get(i) + "," + yrot.get(i) + "," + zrot.get(i) + "," + wrot.get(i) + ",");
					message.append(xpos1.get(i) + "," + zpos1.get(i) + "," + xrot1.get(i) + "," + yrot1.get(i) + "," + zrot1.get(i) + "," + wrot1.get(i));
					
					DataOutputStream out = new DataOutputStream(server.getOutputStream());
					out.writeUTF(message.toString());
					//System.out.println("Data sent");
					
					//******************************************************************************
					//Scheme 1: include a feedback loop to control deviation, this one works
					long time1 = System.currentTimeMillis();
					try
					{	
						Thread.sleep(timeDiff.get(i+1)); 
						
					}catch(InterruptedException e)
					{
						Thread.currentThread().interrupt();
					}
					long timedifference = System.currentTimeMillis()-time1;
					cumtimedifference += (timedifference - timeDiff.get(i+1));
					System.out.println(cumtimedifference);
					if (cumtimedifference > 17)  //one whole frame slower, changed here
					{
						i+=1;
						cumtimedifference -= 17 ;
					}
					
					//*********************************************************************************
					
					//Scheme 2: include a loop to sleep, check time, send data when timestep reached
					/*long time1 = System.currentTimeMillis();
					long cumtimediff = 0;
					while (cumtimediff < timeDiff.get(i+1))
					{
						try
						{	
							Thread.sleep(1); 
						
						}catch(InterruptedException e)
						{
							Thread.currentThread().interrupt();
						}
						cumtimediff = System.currentTimeMillis() - time1;
						System.out.println(cumtimediff);
					}*/
					
					//***********************************************************************************
					//Scheme 3: Combining scheme 1 and scheme 2, does not work well
					
					//scheme 2
					/*long time1 = System.currentTimeMillis();
					long cumtimediff = 0;
					while (cumtimediff < timeDiff.get(i+1))
					{
						try
						{	
							Thread.sleep(1); 
						
						}catch(InterruptedException e)
						{
							Thread.currentThread().interrupt();
						}
						cumtimediff = System.currentTimeMillis() - time1;
						System.out.println(cumtimediff);
					}
					
					//scheme 1
					if (cumtimediff > timeDiff.get(i+1))
					{
						cumtimedifference += (cumtimediff - timeDiff.get(i+1));
						System.out.println("diff" + cumtimedifference);
					}	
					if (cumtimedifference > 16)  //one whole frame slower
					{
						i+=1;
						cumtimedifference -= 16;
					}*/
					
				}
 				//DataInputStream in = new DataInputStream(server.getInputStream());
				//System.out.println(in.readUTF());
				//DataOutputStream out = new DataOutputStream(server.getOutputStream());
				//out.writeUTF(message.toString());
				server.close();
	     }catch(SocketTimeoutException s)
	     {
			System.out.println("Socket timed out");
			break;
	     }catch(IOException e)
	     {
			e.printStackTrace();
    		break;
	     }
	  }
   }

   public static void main(String[] args) throws IOException
   {
       int port = Integer.parseInt(args[0]);
	   getData();
       try
       {
       		Thread t = new tcpserver(port);
 		t.start();
       }catch(IOException e)
       {
		e.printStackTrace();
       }
   }
}
