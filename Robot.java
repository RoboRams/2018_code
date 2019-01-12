/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6488.robot;
import edu.wpi.first.wpilibj.IterativeRobot;

// video functions
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;

// Joystick and motor imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.Talon;

// control pneumatics
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

// Limit switches
import edu.wpi.first.wpilibj.DigitalInput;

// sensor base for the power distribution sensor functionality
import edu.wpi.first.wpilibj.PowerDistributionPanel;

// import for the timer control for autonomous.  
import edu.wpi.first.wpilibj.Timer;


// and, finally, the dash board imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	

	Compressor C;  // define C as a compressor
	Solenoid Solenoid_CubeGrab, 
		     Solenoid_RaiseCube,
		     Solenoid_Bucket;   // define S as a Solenoid (we will have more than one Solenoid!
	Joystick controller1,controller2;
	VictorSP motorL,motorR;  // left and right motor
	VictorSP RightWheel,LeftWheel; // for the dumptruck motors
	Spark cubeH;   // cube handerler motor was wired with a Spark motor controller...
	DifferentialDrive drive;
	
	/* ***************  variables for the timer function  ****************/
	double ElaspsedTime=0,RunTime=2.0;  // measure time from start
	Timer time;
	double autospeed=0.6;    // speed for autonomous
	
	
	/*************************  end timer functions  **************************/
	double lastspeed=0.0;      // velocity ramp initilization
	double lastrotate=0.0;    // rotation speed ramp init
	double limits=0.08;     // limit the accelerateion of the robot
	double limitr=0.08;	   // limit the rotation rate
	
	/****************  a few defined variables   ******************/
	int RightM=0,LeftM=1,chain=2;   // pwm ports for motors  chain == chaindrive for cube hander
	int DumpRight=9,DumpLeft=8;     // pwm ports for the dump motors
	int limitF=9,limitR=0;  // forward and rear limit switch digital io port 
	int sol_grab=0, sol_raise=1,sol_dump=2;  // solenoid ports
	int chain_direction=1;  // 1 for forward, -1 for backward
	
	// initialize the limit switches
    DigitalInput forwardLimitSwitch;
    DigitalInput reverseLimitSwitch;	
    
    PowerDistributionPanel pdp;   // power distribution panel


	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		

		/*************  start camera setting
		 * simple camera set-up.  Only for viewing the usb camera
		 *********************************************************/
		System.out.println("enter thread");
		new Thread(() -> {
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setResolution(160, 120);
            camera.setFPS(20);		  // frames per second	
            camera.setBrightness(50);  // brightness 0-100
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Mat", 160, 120);
            
            Mat source = new Mat();
            Mat output = new Mat();
            System.out.println("before thread.interrupt");
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source,5	);
                 // Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                 Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
                outputStream.putFrame(output);
                // System.out.println("in interupt thread.....");
            }
		}).start();
        /***************   End camera setting  ******************************/
		


        
        /************  start pneumatics settings  **************************/
        C = new Compressor(0);  // setup a new compressor
		C.setClosedLoopControl(true);  
        Solenoid_CubeGrab = new Solenoid(sol_grab);
        Solenoid_RaiseCube = new Solenoid(sol_raise);
        Solenoid_Bucket = new Solenoid(sol_dump);
        
        Solenoid_CubeGrab.set(true);  // close the solenoids
        Solenoid_RaiseCube.set(true);
        Solenoid_Bucket.set(true);
        C.start(); 
        /******************   End pneumatics settings  **********************/
        
        /***************** start drive settings  ****************************/
        motorR = new VictorSP(RightM);
        motorL = new VictorSP(LeftM);
        motorR.setInverted(true);
        motorL.setInverted(true);
        RightWheel = new VictorSP(DumpRight);
        LeftWheel  = new VictorSP(DumpLeft);
        cubeH  = new Spark(chain); 
        cubeH.setInverted(true);
        drive = new DifferentialDrive(motorR,motorL);
	    controller1 = new Joystick(0);    // usb port
	    controller2 = new Joystick(1);
	    /***************** end drive settings   ****************************/
	    
	    /***************** Start Limit Switches  *************************/
	     forwardLimitSwitch = new DigitalInput(limitF);
	     reverseLimitSwitch = new DigitalInput(limitR);	
	    /**************** End Limit Switches *****************************/
	     
	     pdp = new PowerDistributionPanel();  // create instance of a power distribution panel


	}
	
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
		
		/***   create a new timer, reset the timer to zero, and start the timer  *****/
		time = new Timer();
		time.reset();
		time.start();
		Solenoid_RaiseCube.set(true);
		Solenoid_Bucket.set(true);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
				// run for a period of time to, hopefully, cross the line
				ElaspsedTime=time.get();
				if(ElaspsedTime < RunTime )
					drive.tankDrive(autospeed, autospeed);
				else
					drive.stopMotor();
	}

	/**     **************** Start Teleop  **********************************/
	@Override
	public void teleopPeriodic() {
		// boolean solenoid_grab_check,solenodid_raise_check
		// boolean chainOn;
		// double current;
		// boolean pressureSwitch;
		// joystick values
		double speed,speed_chain, rotate;
		/************** ps2 controller mapping.   ****************/ 
		int stick1_h = 0,stick1_v=1;
		int stick2_h=2,stick2_v=3;
		int button1=1,button2=2,button3=3, button4=4;
		
		///   need to map the buttons int button1=
		/**************   end ps2 controler mapping.  I probably should do this more generically */
		

		
		/**************  Drive control  **********************************************/
		speed = controller1.getRawAxis(stick1_v);
		rotate = controller1.getRawAxis(stick2_v);
		
        if ((Math.abs(speed) > Math.abs(lastspeed) ) &&
                (Math.abs(speed - lastspeed) > Math.abs(lastspeed*limits)))
        {
        		if(speed < 0)  speed=lastspeed - limits;  // add or subtract based on the motor direction
                else speed = lastspeed + limits;
        		lastspeed = speed;   // this is the line that was missing
        }
        else lastspeed = speed;

        if ((Math.abs(rotate) > Math.abs(lastrotate) ) &&
                (Math.abs(rotate - lastrotate) > Math.abs(rotate*limitr)))
        {
        		if(rotate < 0)  rotate=lastrotate - limitr;  // add or subtract based on the motor direction
                else rotate = lastrotate + limitr;
        		lastrotate = rotate;   
        }
        else lastrotate = rotate;

    // System.out.println("speed: "+speed+" rotate: "+rotate+" lastvelocity: "+lastvelocity+" axis center: "+center_stick1v_axis);
		
		drive.arcadeDrive(speed,rotate);		
		lastspeed = speed;
		lastrotate = rotate;
		
		// this is for the bucket motors.  I need to check the team code to make sure this is right!
		
		double bspeed = controller2.getRawAxis(stick2_v);
		
		if( bspeed > 0.3 )  bspeed =0.3;  // limit how fast we pull the cube in
		RightWheel.set(-bspeed);  // the motors need to counter rotate.
		LeftWheel.set(bspeed);
		
		
		speed_chain = controller2.getRawAxis(stick1_v); // note, if the cube is raised I have trouble
		 
		
		if( (speed_chain > 0) && forwardLimitSwitch.get())
		{
			System.out.println("in forward limit switch if");
			cubeH.stopMotor();
			chain_direction=1;
		}
		else if( (speed_chain < 0 ) && reverseLimitSwitch.get() )
		{
			System.out.println("in reverse limit swithc if");
			cubeH.stopMotor();
			chain_direction=-1;
		}
		else cubeH.set(speed_chain);
				
		// System.out.println("chain_direction "+chain_direction);
		// System.out.println("forward limit "+forwardLimitSwitch.get()+" reverse "+reverseLimitSwitch.get());
			 	
		// Open and close the solenoids
		Solenoid_CubeGrab.set(controller2.getRawButton(button1));  
		// Solenoid_RaiseCube.set(controller2.getRawButton(button2));
		Solenoid_Bucket.set(controller2.getRawButton(button3));
		
		/// I don't think I need this logic since we are operating in closed loop control
		// if(C.getPressureSwitchValue() == false ) C.stop();
		// else C.start();
		//		boolean enabled = C.enabled();
		//		double current = C.getCompressorCurrent();
		// end status
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
