/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Relay;
/**
* The VM is configured to automatically run this class, and to call the
* functions corresponding to each mode, as described in the IterativeRobot
* documentation. If you change the name of this class or the package after
* creating this project, you must also update the manifest file in the resource
* directory.
*/
public class RobotTemplate extends IterativeRobot
{
    /**
* This function is run when the robot is first started up and should be
* used for any initialization code.
*/

    //These joystick numbers are assigned in order on the drivers station.
    Joystick j1 = new Joystick(3);//Left
    Joystick j2 = new Joystick(1);//Right
    Joystick controller = new Joystick(2);//Xbox

    //motors
    Jaguar fLeft, fRight, bLeft, bRight; //lowerArm, upperArm;
    Victor Elbow, Sholder;

    //input output
    DigitalOutput output; // for ultrasonic
    DigitalInput input;
    Ultrasonic ultraSonic;
    //AxisCamera cam; // camera
    Timer timer = new Timer(); // timer
    DigitalInput left; // for LineTracker
    DigitalInput middle;//for LineTracker
    DigitalInput right;//for LineTracker
    Encoder lowerArmEncoder;//Tracks movement of the lower arm joint

    //What are these for?
    DigitalInput left2, middle2, right2;//
    DigitalInput upper1, upper2, lower1, lower2;
    DigitalInput upperLimitS, lowerLimitS, upperLimitE, lowerLimitE;

    //Drivers Station
    DriverStation ds;
    DriverStationLCD lcd;

    //Pneumatics
    Compressor air;
    Solenoid shifter;//shifts
    Solenoid Kraken;
    Relay minibot, breakOn, breakOff;

    //Variables to remember state
    boolean forkLeft = false;
    boolean pauseAtBegin = false; //Will the robot pause at the beginning of autonomous before moving?
    boolean stopAfterHang = false; //Will the robot stop after it hangs a ubertube?
    boolean turnAfterHang = false; //Will the robot turn as it's backing away, or go straight back? (Assuming that stopAfterHang is false)
    boolean hasHangedTube; // Has the robot hanged its ubertube (or at least attempted to)?
    boolean hasAlreadyPaused; //Has the robot already paused at the beginning? (Assuming that pauseAtBegin is true)
    boolean doneWithAuto; //Has the robot done what it needs to in auto mode?
    boolean usingFork; //Are we taking the forked path?
    boolean upperArmRaised;
    boolean lowerArmRaised;
    boolean atFork = false; // if robot has arrived at fork
    boolean armAtHeight = false;
    int lastSense = 0; // last LineTracker which saw line (1 for left, 2 for right
    int mmDistance;
    //boolean firstrun = true; // ensuring while loop and brake on/off only run once in auto
    int autoState;
    boolean KrakenIsWaiting = false;//prevents multiple firings upon button press


    /*
     * Initializes all robot components
     * This only runs ONCE upon bootup
     */
    public void robotInit()
    {
            try
            {
                //initializes the arm break and sets the direction
                //The single break uses a two relay system, one for on one for off
                breakOn = new Relay(3);
                breakOn.setDirection(Relay.Direction.kForward);
                breakOff = new Relay(2);
                breakOff.setDirection(Relay.Direction.kReverse);

                //unused limit switches (they do not exist)
                //upperLimitS = new DigitalInput(10);
                //lowerLimitS = new DigitalInput(11);
                //upperLimitE = new DigitalInput(12);
                //lowerLimitE = new DigitalInput(13);

                //motors for wheels with PWM channels as arguements
                fLeft = new Jaguar(8); 
                fRight = new Jaguar(6);
                bLeft = new Jaguar(9);
                bRight = new Jaguar(7);
                Sholder = new Victor(3);
                Elbow = new Victor(1);

                //line sensors
                left = new DigitalInput(8); // for LineTracker
                middle = new DigitalInput(6);
                right = new DigitalInput(4);
                //left2 = new DigitalInput(9); // for LineTracker
                //middle2 = new DigitalInput(7);
                //right2 = new DigitalInput(5);

                //the encoder for the lower arm
                lowerArmEncoder = new Encoder(4,11,4,10);

                //ultrasonic sensor init and setup
                output = new DigitalOutput(2); // ultrasonic output
                input = new DigitalInput(3); //ultrasonic input
                ultraSonic = new Ultrasonic(output, input, Ultrasonic.Unit.kMillimeter); //initialize ultrasonic
                ultraSonic.setEnabled(true);
                ultraSonic.setAutomaticMode(true);

                //compressor init
                air = new Compressor(1,1);
                shifter = new Solenoid(8,2);
                shifter.set(false);

                //The soleniod to actuate the claw arm
                Kraken = new Solenoid(8,1);
                Kraken.set(false);

                //the relay to retract the minibot holding pneumatic
                minibot = new Relay(4, 4);
                //minibot.setDirection(Relay.Direction.kForward);
                //minibot.set(Relay.Value.kOn);

                //variables to remeber state. (Are these redundant?)
                hasHangedTube = false;
                hasAlreadyPaused = false;
                doneWithAuto = false;
                autoState = 0;
                upperArmRaised = false;
                lowerArmRaised = false;
                upperArmRaised = false;
                lowerArmRaised = false;

                //starts the encoder
                lowerArmEncoder.start();

                 //drivers station startup
                 ds = DriverStation.getInstance();
                 updateDS();
                 lcd = DriverStationLCD.getInstance();

                 //scam = AxisCamera.getInstance();
                //upperArmEncoder = new Encoder(1,1); //Needs channels
                //lowerArmEncoder = new Encoder(1,2);
                //upperArmEncoder.reset(); //"Zero out" the encoders
               // lowerArmEncoder.reset();
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }

        timer.delay(1);
    }

/*
* This function is called periodically during autonomous
*/
    public void autonomousPeriodic()
    {
        
         shifter.set(true);//sets the shifter in an arbitrary direction so it engages

         if (doneWithAuto)// if autonomous has completed skip the rest
         {
             return;
         }

         forkLeft = ds.getDigitalIn(1);//Go left
         pauseAtBegin = ds.getDigitalIn(2);//pause before moving
         stopAfterHang = ds.getDigitalIn(3);//do not back up after hanging the tube
         turnAfterHang = !stopAfterHang && ds.getDigitalIn(4);//This will only be true if stopAfterHang is false
         usingFork = ds.getDigitalIn(5);//Should the robot expect a fork?
         
         //update the compressor and drivers station
         updateComp();
         updateDS();
         
         //this is for debugging
         if(hasHangedTube)
            System.out.println("hung");

         //Stores the linetracker data for this iteration
         boolean leftValue = left.get();
         boolean middleValue = middle.get();
         boolean rightValue = right.get();

         //Why are these here?
         //boolean leftValue2 = left2.get();
         //boolean middleValue2 = middle2.get();
         //boolean rightValue2 = right2.get();
         
         //This computes an int to represent a state in the later switch that
         //raises the arm to the correct height based off the DS values
         int height =   (int)(ds.getDigitalIn(5)?1:0)+
                        (int)(ds.getDigitalIn(6)?2:0)+
                        (int)(ds.getDigitalIn(7)?4:0)+
                        (int)(ds.getDigitalIn(8)?8:0);

       
        double speed = 0.3;//This is the base speed for autonomous

        //This computes an int to represent a state in the later switch that
        //follows the line based off the line sensor values
        int lineState = (int)(rightValue?0:1)+
                        (int)(middleValue?0:2)+
                        (int)(leftValue?0:4);


        //vegetable robot
        if (ds.getAnalogIn(2) > 0)
            {
                return;
            }


        //raises arm for autonomous, it only runs once
        if(!armAtHeight)
        {
            System.out.println("Moving arm");
            setArmHeight(height);
            armAtHeight = true;
        }
        
        //no line following, just a forward movement
        if (ds.getAnalogIn(1) > 0)
        {
            //dead reckoning
            //BE SURE TO RAISE ARM BC THIS IS HIGH LOW
            //^That needs to be told to the drivers
            setArmHeight(height);
            straight(speed);//move until close enough
            if(closerThan(mmDistance))
            {
                straight(0); //Stop
                hangTube();
                return;
            }
        }

        if (hasHangedTube && !turnAfterHang) //If the robot has hanged the tube, and then should back straight up...
            {
                straight(-speed); // Back straight up
                try
                {
                    Thread.sleep(2000); //And after two seconds...
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                straight(0); //Stop backing up
                doneWithAuto = true;
                return;
            }
        else if (hasHangedTube && turnAfterHang) //If the robot has hanged the tube, and then should turn around...
            {
                straight(-speed); //Back straight up
                try
                {
                    Thread.sleep(2000); //And after two seconds...
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                hardRight(speed); //Start turning around
                try
                {
                    Thread.sleep(3000);
                    while(!middleValue)
                        Thread.sleep(50);
                    straight(0);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                straight(0); //Stop turning around
                doneWithAuto = true;
                return;
         }
        else if (!hasHangedTube && ds.getAnalogIn(1) == 0)//follow the line
        {
             moveWhileTracking(lineState, speed, autoState);
        }


        if(closerThan(mmDistance))//hang the tube when at the right distance
        {
            straight(0); //Stop
            hangTube();
            return;
        }

        if (pauseAtBegin && !hasAlreadyPaused) //If the robot should pause at the beginning and it hasn't already paused...
        {
            try
            {
                straight(0);
                Thread.sleep(3000); //Pause for 3 seconds
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }

            hasAlreadyPaused = true; //The robot has now paused
        }

    }
    
    public void teleopPeriodic()
    {

        //System.out.println(ultraSonic.getRangeMM());
        System.out.println(lowerArmEncoder.get() + " counts" + (lowerArmEncoder == null));//this is for debugging

        //since we are running pwm's instead of CAN, this won't do anything
        try{
        setCoast(fLeft); // set them to drive in coast mode (no sudden brakes)
        setCoast(fRight);
        setCoast(bLeft);
        setCoast(bRight);
        //setBreak(lowerArm);
        //setBreak(upperArm);
        }catch (Exception e) {}

        if(j1.getRawButton(7))//resets the encoder on the arm to 0, use in auto testing
        {
             //Elbow.set(-0.5); // TO BE EXPERIMENTED
               //     try
                 //   {
                   //     Thread.sleep(200);
                    //} catch (Exception e) {e.printStackTrace();}
                    //Elbow.set(0);
            lowerArmEncoder.reset();
        }

       
         if(j1.getRawButton(6))//reset arm status for auto tests
        {
            armAtHeight = false;
        }

        //updates all other systems
        updateComp();
        updateGear();
        updateDS();

        //On pressing and releasing a button the claw actuates
        if(controller.getRawButton(6))
        {
            KrakenIsWaiting = true;
        }
        else if(KrakenIsWaiting)
        {
            changeKraken();
            KrakenIsWaiting = false;
        }
        
        //updates the arm heights
        updateLowerArm();
        updateUpperArm();

        //One of these works
        //TEST IT TO FIND THE RIGHT ONE!!!
        if(j1.getRawButton(10) && j2.getRawButton(10))
        {
            System.out.println("Mini1");
            minibot.set(Relay.Value.kForward);
        }

        if(j1.getRawButton(9) && j2.getRawButton(9))
        {
            System.out.println("Mini1");
            minibot.set(Relay.Value.kOff);
            //minibot = new Relay(4,4);
        }
      /*  else
        {
             minibot.set(Relay.Value.kForward);
        }*/

       /* if(j1.getRawButton(9) && j2.getRawButton(8) && j1.getThrottle() > 0)
        {
            System.out.println("Mini1");
            minibot.set(Relay.Value.kForward);
        }*/


       
         setLefts(deadzone(-j1.getY()));
         setRights(deadzone(-j2.getY()));
        
    }

    //sets the left wheel motors
    private void setLefts(double d)
    {
        try
        {
        fLeft.set(d);
        bLeft.set(d);

        }
        catch (Exception e)
        {
            DriverStationLCD lcd = DriverStationLCD.getInstance();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "NOT A CAN EXCEPTION");
            lcd.updateLCD();
        }
    }

    //Sets the right wheel motors
    private void setRights(double d)
    {
        try{
        fRight.set(-d);
        bRight.set(-d);
        } catch (Exception e){
            e.printStackTrace();
            DriverStationLCD lcd = DriverStationLCD.getInstance();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "NOT A CAN EXCEPTION!!!");
            lcd.updateLCD();
        }
    }

   //update the driver's station outputs
    private void updateDS()
    {
        ds.setDigitalOut(1, forkLeft);
        ds.setDigitalOut(2, pauseAtBegin);
        ds.setDigitalOut(3, stopAfterHang);
        ds.setDigitalOut(4, turnAfterHang);
        ds.setDigitalOut(5, shifter.get());
        ds.setDigitalOut(2, pauseAtBegin);
        ds.setDigitalOut(3, stopAfterHang);
        ds.setDigitalOut(4, turnAfterHang);
        ds.setDigitalOut(5, shifter.get());
    }

   
    //this does not work with PWM
    public void setCoast(Jaguar jag) throws CANTimeoutException
    {//Sets the drive motors to coast mode
       //; try{jag.configNeutralMode(Jaguar.NeutralMode.kCoast);} catch (Exception e) {e.printStackTrace();}
    }

    //Lucas Mark 1

    public void updateComp()
    {
        if (air.getPressureSwitchValue())
            air.stop();
        else
            air.start();
    }

    boolean switchStateShift = false;
    public void updateGear()
    {

        if(j1.getTrigger())
        {
            shifter.set(true);
        }
        else if(j2.getTrigger())
        {
            shifter.set(false);
        }

    }

     public void setBreak(Jaguar jag) throws CANTimeoutException
    {//Sets the drive motors to brake mode
       // try{jag.configNeutralMode(CANJaguar.NeutralMode.kBrake);} catch (Exception e) {e.printStackTrace();}
    }

    public double deadzone(double d)
    {//deadzone for input devices
        if (Math.abs(d) < .3) {
            return 0;
        }
        return d / Math.abs(d) * ((Math.abs(d) - .3) / .7);
    }
    //comment
     public double tinyDeadzone(double d)
    {//deadzone for input devices
        if (Math.abs(d) < .05) {
            return 0;
        }
        return d / Math.abs(d) * ((Math.abs(d) - .05) / .95);
    }

    public void straight(double speed)
    {
        setLefts(speed);
        setRights(speed);
    }

    public void hardLeft(double speed)
    {
        setLefts(-speed);
        setRights(speed);
    }

    public void hardRight(double speed)
    {
        setLefts(speed);
        setRights(-speed);
    }

    public void softLeft(double speed)
    {
        setLefts(0);
        setRights(speed);
    }

    public void softRight(double speed)
    {
        setLefts(speed);
        setRights(0);
    }

    int lastRange = 0; // ascertains that you are less than 1500mm
    public boolean closerThan(int millimeters)
    {
        if (ultraSonic.isRangeValid() && ultraSonic.getRangeMM() < millimeters)
        {
            if (lastRange > 4) // 4 checks to stop
            {
                return true;
            }
            else lastRange++;
        }
        else
        {
            lastRange = 0;
        }
        return false;

    }

    public void updateLowerArm()
    {
        //System.out.println(deadzone(-controller.getY()));
        if(deadzone(-controller.getY()) == 0.0)
        {
            breakOn.set(Relay.Value.kOn);
            breakOff.set(Relay.Value.kOff);
        }
        else
        {
            breakOff.set(Relay.Value.kOn);
            breakOn.set(Relay.Value.kOff);
        }
        //if(deadzone(controller.getY()) > 0) this may work now
         //   Sholder.set(0.5*deadzone(-controller.getY()));
       // else
       // System.out.println(deadzone(-controller.getY()));
            Sholder.set(deadzone(-controller.getY()));
        
       // Elbow.set(deadzone(0.4*controller.getY()));
            
    }

    public void updateUpperArm()
    {
        Elbow.set(deadzone(controller.getRawAxis(5)));

    }
    boolean run = true;

    public void changeKraken()
    {
        Kraken.set(!Kraken.get());
    }


    int waitFor = 0;
    public void moveWhileTracking(int lineState, double speed, int autoState)
    {

        if(run)
        {
        switch (lineState)
        {

            case 0: //No sensors see the line
                System.out.println("Lost the line: " + lastSense);
                speed = .5;
                if(waitFor == 1)
                {
                    hardLeft(speed);
                }
                else if(waitFor == 2)
                {
                    hardLeft(speed);
                }
                else if (lastSense == 1) // left is last seen, go left
                {
                    setLefts(-speed);//speed * 0.7);
                    setRights(speed);
                }
                else if (lastSense == 2) // go right
                {
                    setLefts(speed);
                    setRights(-speed);//speed * 0.7);
                }
                else
                {
                    setLefts(0.2); // CAUTION! Go Slowly!
                    setRights(0.2);
                    System.out.println("Panic!");
                }
                break;
            case 1: //Right sees the line
                softRight(speed);
                lastSense = 2;
                break;
            case 2: //Middle sees the line
                straight(speed);
                break;
            case 3: //Middle and right sees the line
                softRight(speed);
                lastSense = 2;
                break;
            case 4: //Left sees the line
               // System.out.println("Hard left");
                softLeft(speed);
                lastSense = 1;
                break;
            case 5: //Left and right see the line
                System.out.println("At Cross");
                if(forkLeft)
                {
                    hardLeft(speed);
                    waitFor = 1;
                }
                else
                {
                    hardRight(speed);
                    waitFor = 2;
                }
                break;
            case 6: //Left and middle see the line
                softLeft(speed);
                lastSense = 1;
                break;
            case 7: //All three see the line
                System.out.println("At Cross 7");
                if(forkLeft)
                {
                    hardLeft(speed);
                }
                else
                {
                    hardRight(speed);
                }
                break;
            default:
                System.out.println("You're doomed. Run.");
                break;
        }
        }
    }

    public boolean breaking()
    {
        //System.out.println(j1.getRawButton(3) + " : " + j2.getRawButton(3) + " : " + j2.getRawButton(3));
        if(j1.getRawButton(3) || j2.getRawButton(3))
        {
           try{
           setBreak(fLeft);
           setBreak(bLeft);
           setBreak(fRight);
           setBreak(bRight);

           }catch(CANTimeoutException e){
DriverStationLCD lcd = DriverStationLCD.getInstance();
lcd.println(DriverStationLCD.Line.kMain6, 1, "Breaking failed");
lcd.updateLCD();
            return false;
        }
        }
        else if (j1.getRawButton(2) || j2.getRawButton(2))
        {
             try{
           setBreak(fLeft);
           setBreak(bLeft);
           setBreak(fRight);
           setBreak(bRight);

           }catch(CANTimeoutException e){
DriverStationLCD lcd = DriverStationLCD.getInstance();
lcd.println(DriverStationLCD.Line.kMain6, 1, "Breaking failed");
lcd.updateLCD();}

             straight(0);
             return true;
        }
        else {
            try{
           setCoast(fLeft);
           setCoast(bLeft);
           setCoast(fRight);
           setCoast(bRight);

           }catch(CANTimeoutException e){
DriverStationLCD lcd = DriverStationLCD.getInstance();
lcd.println(DriverStationLCD.Line.kMain6, 1, "Breaking failed");
lcd.updateLCD();
        }
        }
        return false;
    }

    public void hardBreak()
    {

    }

    public void hangTube() //assumes bot is already there at the rack
    {

         Kraken.set(true);
                        try
                        {
                            Thread.sleep(500); //And after 1/2 seconds...
                        }
                        catch (Exception e)
                        {
                            e.printStackTrace();
                        }
                        straight(-.6);

                        try
                        {
                            Thread.sleep(2000); //And after two seconds...
                        }
                        catch (Exception e)
                        {
                            e.printStackTrace();
                        }

                        straight(0);

                        hasHangedTube = true;


    }

    public void setArmHeight(int height)
    {
        System.out.println("ArmHeightMethod : " + height);
        boolean lowerArmRaised = false;
        boolean upperArmRaised = false;
        int DoNotUse = 0;
          switch(height){
                default:
                    Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                    {
                        Thread.sleep(450);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);
                    mmDistance = 665;
                        break;
                case 5:
                     Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                    {
                        Thread.sleep(250);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);
                    breakOff.set(Relay.Value.kOn);
                    breakOn.set(Relay.Value.kOff);
                    while(lowerArmEncoder.get() < DoNotUse)//low Middle
                    {
                        updateComp();
                        updateDS();
                        Sholder.set(-1);
                    }
                    breakOff.set(Relay.Value.kOff);
                    breakOn.set(Relay.Value.kOn);
                    while(!upperArmRaised)
                    {
                        //timer here if needed
                        upperArmRaised = true;

                    }
                    break;
                case 9:
                    System.out.println("Case 9");
                     Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                    {
                        Thread.sleep(250);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);
                    //low high
                    breakOff.set(Relay.Value.kOn);
                    breakOn.set(Relay.Value.kOff);
                    while(lowerArmEncoder.get() < 420)
                    {
                         updateComp();
                         updateDS();
                        System.out.println("Encoder");
                        Sholder.set(-.8);
                    }
                    breakOff.set(Relay.Value.kOff);
                    breakOn.set(Relay.Value.kOn);
                    while(!upperArmRaised)
                    {
                        //timer here if needed
                        upperArmRaised = true;

                    }
                    mmDistance = 760;
                    break;
                case 2:
                    //high low
                    Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                    {
                        Thread.sleep(450);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);

                    Sholder.set(-0.8);
                    try {Thread.sleep(300);} // 0.3s for shoulder
                    catch (Exception e) {e.printStackTrace();}
                    Sholder.set(0);
                        break;

                case 4:
                     Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                    {
                        Thread.sleep(250);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);
                    //high middle
                    breakOff.set(Relay.Value.kOn);
                    breakOn.set(Relay.Value.kOff);
                    while(lowerArmEncoder.get() < DoNotUse)
                    {
                        updateComp();
                        updateDS();
                        Sholder.set(-.8);
                    }
                    breakOff.set(Relay.Value.kOff);
                    breakOn.set(Relay.Value.kOn);
                    while(!upperArmRaised)
                    {
                        //timer here if needed
                        upperArmRaised = true;

                    }
                    break;
                case 8:
                      Elbow.set(-0.5); // TO BE EXPERIMENTED
                    try // for 0.3s
                              
                    {
                        Thread.sleep(250);
                    } catch (Exception e) {e.printStackTrace();}
                    Elbow.set(0);
                    //high high
                    
                   // if (firstrun)
                    
                            breakOff.set(Relay.Value.kOn);
                            breakOn.set(Relay.Value.kOff);
                           // firstrun = false;



                        while(lowerArmEncoder.get() < 430)
                        {
                            DriverStationLCD lcd = DriverStationLCD.getInstance();
                            lcd.println(DriverStationLCD.Line.kMain6, 1, "I'm trying, I really am!!!");
                            System.out.println("Encoder:" + lowerArmEncoder.get());

                            lcd.updateLCD();
                            updateComp();
                             updateDS();
                            Sholder.set(0.3);

                        }
                        Sholder.set(0);
                        breakOff.set(Relay.Value.kOff);
                        breakOn.set(Relay.Value.kOn);
                        if(!upperArmRaised)
                        {
                            //timer here if needed
                            upperArmRaised = true;

                        }

                    break;
                }
    }

    public int countToDistS()
    {
        return 0; //lowerArmEncoder.get();
    }

    private double countToDistE()
    {
        return 0;//upperArmEncoder.get();
    }

}

