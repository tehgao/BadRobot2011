    /*----------------------------------------------------------------------------
/* Copyright (c) FIRST 2008. All Rights Reserved.
/* Open Source Software - may be modified and shared by FRC teams. The code
/* must be accompanied by the FIRST BSD license file in the root directory of
/* the project.
/*----------------------------------------------------------------------------*/
package code.badrobot;

import code.badrobot.tracking.Target;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.CANJaguar;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class BadRobot extends IterativeRobot {
    long time;
    long superAwesomeTimeVariable=-1;

    long timeVariable=-9001;

    boolean timing=false;
    final boolean DEBUG_MODE = false;//debugging variable, because we don't have #DEFINE
/**
    final int ENCODER1_A_CHANNEL = 1; //On Front Left
    final int ENCODER1_B_CHANNEL = 2;
    final int ENCODER2_A_CHANNEL = 3; //On Front Right
    final int ENCODER2_B_CHANNEL = 4;
    final int ENCODER3_A_CHANNEL = 5; //On Back Left
    final int ENCODER3_B_CHANNEL = 6;
    final int ENCODER4_A_CHANNEL = 7; //On Back Right
    final int ENCODER4_B_CHANNEL = 8;
*/
// jaguar ID nodes
    boolean pneumEnab = true;


    final boolean ACCESSORIES = false;

    final int fLeft_ID = 10,    //CAN IDs
    fRight_ID = 4,
    bLeft_ID = 9,
    bRight_ID = 7,
    armID = 5;

    final int winch_ID = 1; //winch (PWM) ID


// relay channels
    final int ROLLER1_RELAY_CHANNEL = 5,
    ROLLER2_RELAY_CHANNEL = 8,
    PRESSURE_GUAGE = 1,
    COMPRESSOR_RELAY = 1,
    SOLENOID_LEFT = 1,
    SOLENOID_RIGHT = 2;
/*    OPEN_SWITCH_CHANNEL = 7,//FIX ME!!!!!!
    CLOSE_SWITCH_CHANNEL = 8;//FIX ME TOO!!!!!!!!!!!!
*/

// States
    final int CLOSED = 0;
    final int OPENING = 1;
    final int OPENED = 2;
    final int CLOSING = 3;
    final int DONOTHING = 9004;

    int kickerState = CLOSED;
    int rollerState = CLOSED;
    int armState = CLOSED;
    int armGrabState = CLOSED;
    int pneumState = OPENED;

    int autoState = 0;


    Timer t = new Timer();//this should be doing timing crap, hopefully it is, fix this comment when tested
    CANJaguar fLeft = new CANJaguar(fLeft_ID);
    CANJaguar fRight = new CANJaguar(fRight_ID);
    CANJaguar bLeft = new CANJaguar(bLeft_ID);
    CANJaguar bRight = new CANJaguar(bRight_ID);
    CANJaguar armMotor = new CANJaguar(armID);

    Victor winch = new Victor(winch_ID);

    Solenoid solL = new Solenoid(SOLENOID_LEFT);
    Solenoid solR = new Solenoid(SOLENOID_RIGHT);

    Relay roller1 = new Relay(ROLLER1_RELAY_CHANNEL);
    Relay roller2 = new Relay(ROLLER2_RELAY_CHANNEL);
    //Relay kickPneum1 = new Relay(KICK1_RELAY_CHANNEL);
    //Relay kickPneum2 = new Relay(KICK2_RELAY_CHANNEL);




    Joystick jLeft = new Joystick(1);
    Joystick jRight = new Joystick(2);
    Joystick ps2 = new Joystick(3);
/**
    Encoder fLeftEnc = new Encoder(ENCODER1_A_CHANNEL, ENCODER1_B_CHANNEL);
    Encoder fRightEnc = new Encoder(ENCODER2_A_CHANNEL, ENCODER2_B_CHANNEL);
    Encoder bLeftEnc = new Encoder(ENCODER3_A_CHANNEL, ENCODER3_B_CHANNEL);
    Encoder bRightEnc = new Encoder(ENCODER4_A_CHANNEL, ENCODER4_B_CHANNEL);
*/
    Compressor airComp = new Compressor(PRESSURE_GUAGE,COMPRESSOR_RELAY);

    Watchdog feedMe = getWatchdog();//feed him

    AxisCamera camera;

    Double randomTimeVar;//has suprisingly low entropy given its alleged randomness

    boolean kickButton,
    armButtonUp=false,
    armButtonDown=false,
    winchButton,
    rollerButton,
    armGrabButton,
    airCompToggle;

    //   Gyro gyro = new Gyro(GYROSCOPE_CHANNEL);
    public void robotInit()
    {//called once when robot is turned on
        //DISABLE CAMERA INITIALIZATION IF THE CAMERA IS NOT CONNECTED
	feedMe.setExpiration(1);
	feedMe.feed();

	timing = false;

        setCoast(fLeft); //jaguars are set to coast regardless of jumper setting
        setCoast(fRight);
        setCoast(bLeft);
        setCoast(bRight);
        setCoast(armMotor);

        camera = AxisCamera.getInstance();
        t.start();//ensures timer is init'd, instantiation might do it, but this makes sure
        airComp.start();
        //       gyro.reset();
    }

    public void autonomousInit()
    {
	autoState=0;
	feedMe.feed();
	superAwesomeTimeVariable=-1;
    }

    public void autonomousPeriodic()
    {//loops during autonomous
	updateAirComp(airComp);
	if(superAwesomeTimeVariable==-1)
	{
	    superAwesomeTimeVariable=-t.getUsClock();
	}
	long delta = superAwesomeTimeVariable+t.getUsClock();
	if(DEBUG_MODE)
	{
	    System.out.print("AUTOSTATE = "+autoState+'\n');
	}
	feedMe.feed();
	if(delta<5000000)
	{
	    //compress air and the such
	    setLeftMotors(0);
	    setRightMotors(0);
	}
	else if(delta<6000000)
	{
	    setLeftMotors(-.5);
	    setRightMotors(-.5);
	    if(delta<5750000)//t>5s, <6.5s (<1.5s elapsed driving)
	    {
		solL.set(false);
		solR.set(false);
	    }
	    else//t>6.5s, <7s (>1.5s driving)
	    {
		solL.set(true);
		solR.set(true);
	    }
	}
	else
	{
	    setLeftMotors(0);
	    setRightMotors(0);
	    solL.set(false);
	    solR.set(false);
	}













/**state machine!!!! dead now
	if(autoState==0)
	{
	    System.out.println("autoState==1");
	    if(superAwesomeTimeVariable<0)
	    {
		superAwesomeTimeVariable=t.getUsClock();
	    }
	    if(superAwesomeTimeVariable>10000000)
	    {
		autoState++;
	    }
	}
        if(autoState==1)
	{
	    System.out.println("autoState==1");
	    setLeftMotors(-.5);
	    setRightMotors(-.5);
	    superAwesomeTimeVariable=t.getUsClock();
	    autoState++;
	}
	else if(autoState==2)
	{
	    setLeftMotors(-.5);
	    setRightMotors(-.5);
	    if(t.getUsClock()-superAwesomeTimeVariable>1000000)
	    {
		System.out.println("t.getUsClock()-superAwesomeTimeVariable>500000 = ");
		solL.set(true);
		solR.set(true);
		autoState++;
	    }
	}
	else if(autoState==3)
	{
	    solL.set(true);
	    solR.set(true);
	    if(t.getUsClock()-superAwesomeTimeVariable>1000000)
	    {
		System.out.println("t.getUsClock()-superAwesomeTimeVariable>1000000 = ");
		solL.set(false);
		solR.set(false);
		autoState++;
		setRightMotors(0);
		setLeftMotors(0);
	    }
	}
	else
	{
	    solL.set(false);
	    solR.set(false);
	}
	updateAirComp(airComp);


	/**if (DEBUG_MODE) {
            System.out.println("AUTONOMOUS LOOP");
        }
        updateAirComp(airComp);
        randomTimeVar = new Double(t.get());

        if (autoState==1)
        {
            if (t.get()-randomTimeVar.doubleValue()<2)
            {
                setRightMotors(0.2);
                setLeftMotors(0.2);
            }
            else
            {
                findTarget();
            }
        }
        else if (autoState==2)
        {
	    autoState++;
	    solL.set(true);
	    solR.set(true);
	    superAwesomeTimeVariable=t.getUsClock();
        }
        else
        {
	    if(t.getUsClock()-superAwesomeTimeVariable>2000)
	    {
		solL.set(false);
		solR.set(false);
	    }
	}
        /*
        if(randomTimeVar == null)//if time of start isn't defined...
        {
        randomTimeVar = new Double(t.get());//define time of auton start
        }
        if(t.get() - randomTimeVar.doubleValue() < 5)//if auton has been going <5 seconds...
        {
        aux.set(.05);//run aux motor, just a test
        feedMe.feed();//feed me!
        }
        else if(t.get() - randomTimeVar.doubleValue() < 10)//auton time >5s and <10s
        {
        aux.set(0);//turn aux motor off
        feedMe.feed();//feed me!
        }
        else//>10 sec auton
        {
        aux.set(-.05);//turn aux motor back; test
        feedMe.feed();//feed me!
        }
         */
    }

    public void teleopInit()
    {/*can be used to do anything once any time that the robot is brought into teleop*/
	feedMe.setExpiration(5d);
	feedMe.feed();
    }

    public void teleopPeriodic()
    {//loops during teleop
        try{
	if (DEBUG_MODE)
	{
            System.out.println("TELEOP LOOP");
        }
        updateDriveMotors();
        updateAirComp(airComp);
        //updateRollers();

	updateKicker();
        updateArm();
        DriverStationLCD.getInstance().updateLCD();


        feedMe.feed();//feed me!
	}
	catch(Exception e)
	{
	    feedMe.feed();
	    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kMain6, fRight_ID, "EXCEPTION!!!!!!\n"+e.getMessage());
	}
    }

    public double deadzone(double d)
    {//deadzone for input devices
        if (Math.abs(d) < .05) {
            return 0;
        }
        return d / Math.abs(d) * ((Math.abs(d) - .05) / .95);
    }


    /**
    Sets the speed controller values
    If the top joystick button is pressed, robot switches into two wheel drive
    Left top activates front two wheel drive
    Right top activates back two wheel drive
     */
    public void updateDriveMotors()
    {//sets the motors to their appropriate values
	feedMe.feed();
        if (jLeft.getButton(Joystick.ButtonType.kTop)) // front two wheels
        {
            fLeft.set(jLeft.getY());
            fRight.set(jRight.getY());
            bLeft.set(0);
            bRight.set(0);
        }
        else if (jRight.getButton(Joystick.ButtonType.kTop)) // back two wheels
        {
            bLeft.set(jLeft.getY());
            bRight.set(jRight.getY());
            fLeft.set(0);
            fRight.set(0);
        }
        else
        {
            setLeftMotors(jLeft.getY());
            setRightMotors(jRight.getY());
        }
    }

    public void setLeftMotors(double d)
    {//sets left motors to their appropriate values
        fLeft.set(-d);
        bLeft.set(-d);
        if (DEBUG_MODE)
        {
            System.out.println("FLEFT: " + fLeft.get() + ", BLEFT: " + bLeft.get());
        }
    }

    public void setRightMotors(double d)
    {//sets right motors to their appropriate values
        fRight.set(d);
        bRight.set(d);
        if (DEBUG_MODE)
        {
            System.out.println("FRIGHT: " + fRight.get() + ", BRIGHT: " + bRight.get());
        }
    }

    public void setCoast(CANJaguar jag)
    {//Sets the drive motors to coast mode
        jag.configNeutralMode(CANJaguar.NeutralMode.kCoast);
    }

    public void updateAirComp(Compressor comp)
    {//updates Compressor comp to the either run or stop
	feedMe.feed();
	if (comp.getPressureSwitchValue())
	{
	    comp.stop();
	    return;
	}
	else
	{
	    comp.start();
	}
    }

    public void updateArm()
    {//state machine for the arm
	if(ps2.getRawButton(4))
	{
	    armMotor.set(1);
	}
	else if(ps2.getRawButton(2))
	{
	    armMotor.set(-1);
	}
	else
	{
	    armMotor.set(0);
	}
	if(ps2.getRawButton(3)&&ps2.getRawButton(1))
	{
	    winch.set(-.2);
	}
	else if(ps2.getRawButton(3))
	{
	    winch.set(1);
	}
	else
	{
	    winch.set(0);
	}
	feedMe.feed();
    }

    public void updateKicker()
    {//state machine for the kicker
	/**if(DEBUG_MODE)System.out.println("KICK FUNCTION ps2.getRawButton(6)=="+ps2.getRawButton(6));
        if(!timing&&ps2.getRawButton(6))
	{
	    if(DEBUG_MODE)System.out.println("KICKING");
	    solL.set(true);
	    solR.set(true);
	    time = Timer.getUsClock();
	    timing=true;
	}
	else
	{
	    if(Timer.getUsClock()-time>1000000&&timing)
	    {solL.set(false);
	    solR.set(false);
	    timing=false;
	    if(DEBUG_MODE)System.out.println("RETRACTING");}
	}*/
	System.out.println("KICKING");
	solL.set(ps2.getRawButton(6)||jRight.getRawButton(1));
	solR.set(ps2.getRawButton(6)||jRight.getRawButton(1));
	feedMe.feed();
    }

    public void findTarget()
    {//Looks for targets will spin in an arc for a specified time
        int findState = 0; //State of camera finder
        int sweepNumber = 0;
        double timeGiven = 5; //Time allowed before state changes
        DriverStationLCD.getInstance().updateLCD();
        Double timeVar = new Double(t.get()); //time when findTarget() is called

        if (camera.freshImage())
        {
            try {
                ColorImage image = camera.getImage();
                Thread.yield();
                Target[] targets = Target.findCircularTargets(image);
                Thread.yield();
                image.free();
                if (targets.length == 0 || targets[0].m_score < 0.1)
                {
                    if (sweepNumber < 5)
                    {
                        if (findState == 0)
                        {
                            setRightMotors(0.1);
                            setLeftMotors(-0.1);
                            targets = Target.findCircularTargets(image);
                            if (t.get() - timeVar.doubleValue() > timeGiven)
                            {
                                findState = 1;
                                timeVar = new Double(t.get());
                                sweepNumber++;
                            }
                        }
                        else
                        {
                            setRightMotors(-0.1);
                            setLeftMotors(0.1);
                            targets = Target.findCircularTargets(image);
                            if (t.get() - timeVar.doubleValue() > timeGiven)
                            {
                                findState = 0;
                                timeVar = new Double(t.get());
                                sweepNumber++;
                            }
                        }
                    }
                    else // break after five sweeps
                    {
                        setRightMotors(0);
                        setLeftMotors(0);
                    }
                }
                else
                {
		    if(Math.abs(targets[0].getHorizontalAngle())<15)/////CHANGE THIS VALUE!!!!
		    {
                    setRightMotors(0);
                    setLeftMotors(0);
		    autoState++;
		    }
		    System.out.println(targets[0]);
                    System.out.println("Target Angle: " + targets[0].getHorizontalAngle());
                }
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            } catch (AxisCameraException ex) {
                ex.printStackTrace();
            }
        }
    }
}
