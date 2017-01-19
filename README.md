# OpenDS-changes-for-self-driving

We are making several changes to the Java programs for OpenDS . Other changes include . 


OpenDS_eclipse.txt - How to run OpenDS on Eclipse.
tcpserver.java - Push driving data from a text file to canClient.java to enable autonomous mode. 

Changes include:

1. Self-driving of steering or user cars : Use tcpserver.java to run autonomous mode.
The canClient can be enabled from assets/DrivingTasks/Projects/$<projectname>$/settings.xml.

	Set <CANInterface>
		<enableConnection>false</enableConnection> to true.
    
	Run openDS with this option and tcpserver.java simultaneously.

2. Access driving data from the moving obstacles in the simulation
To log new variables from driving tasks modify DataWriter.java under analyzer class in the source code

3. Making the dashboard appropriate for US driving
Modified PanelCenter.java to add windshield 
Change speed and RPM display from here during autonomous mode
