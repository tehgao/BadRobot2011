/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.CANJaguar;
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

    Joystick j1 = new Joystick(2);
    Joystick j2 = new Joystick(3);
    Joystick controller = new Joystick(1);
    CANJaguar fLeft, fRight, bLeft, bRight; //lowerArm, upperArm; //motors
    Victor Elbow, Sholder;
    DigitalOutput output; // for ultrasonic
    DigitalInput input;
    Ultrasonic ultraSonic;
    AxisCamera cam; // camera
    Timer timer = new Timer(); // timer
    DigitalInput left; // for LineTracker
    DigitalInput middle;
    DigitalInput right , left2, middle2, right2;
    DigitalInput upper1, upper2, lower1, lower2;
    DriverStation ds;
    Compressor air;
    Solenoid shifter;//shifts


    Solenoid Kraken, break1, break2, minibot;

    boolean forkLeft = false;
    boolean pauseAtBegin = false; //Will the robot pause at the beginning of autonomous before moving?
    boolean stopAfterHang = false; //Will the robot stop after it hangs a ubertube?
    boolean turnAfterHang = false; //Will the robot turn as it's backing away, or go straight back? (Assuming that stopAfterHang is false)
    boolean hasHangedTube; // Has the robot hanged its ubertube (or at least attempted to)?
    boolean hasAlreadyPaused; //Has the robot already paused at the beginning? (Assuming that pauseAtBegin is true)
    boolean doneWithAuto; //Has the robot done what it needs to in auto mode?
    boolean usingFork; //Are we taking the forked path?
    DriverStationLCD lcd;
    boolean upperArmRaised;
    boolean lowerArmRaised;

    DigitalInput upperLimitS, lowerLimitS, upperLimitE, lowerLimitE;


    Encoder upperArmEncoder;
    Encoder lowerArmEncoder;

    int autoState;

    public void robotInit()
    {
            try
            {

                upperLimitS = new DigitalInput(10);
                lowerLimitS = new DigitalInput(11);
                upperLimitE = new DigitalInput(12);
                lowerLimitE = new DigitalInput(13);

                fLeft = new CANJaguar(10); // motors for wheels with CAN ports as arguements
                fRight = new CANJaguar(4);
                bLeft = new CANJaguar(9);
                bRight = new CANJaguar(7);
                Sholder = new Victor(1);
                Elbow = new Victor(3);

                left = new DigitalInput(8); // for LineTracker
                middle = new DigitalInput(6);
                right = new DigitalInput(4);
                //left2 = new DigitalInput(9); // for LineTracker
                //middle2 = new DigitalInput(7);
                //right2 = new DigitalInput(5);

                output = new DigitalOutput(2); // ultrasonic output
                input = new DigitalInput(3); //ultrasonic input
                ultraSonic = new Ultrasonic(output, input, Ultrasonic.Unit.kMillimeter); //initialize ultrasonic
                ultraSonic.setEnabled(true);
                ultraSonic.setAutomaticMode(true);

                air = new Compressor(1,1);
                shifter = new Solenoid(8,1);
                shifter.set(false);

                Kraken = new Solenoid(8,2);
                Kraken.set(false);
                break1 = new Solenoid(8,3);
                break2 = new Solenoid(8,4);
                minibot = new Solenoid(8,5);
                minibot.set(true);
                ds = DriverStation.getInstance();
                hasHangedTube = false;
                hasAlreadyPaused = false;
                doneWithAuto = false;
                autoState = 0;
                updateDS();


                upperArmRaised = false;
                lowerArmRaised = false;

                lcd = DriverStationLCD.getInstance();

                upperArmRaised = false;
                lowerArmRaised = false;

                lcd = DriverStationLCD.getInstance();

                cam = AxisCamera.getInstance();

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

    /**
    * This function is called periodically during autonomous
    */

    boolean atFork = false; // if robot has arrived at fork
    boolean armAtHeight = false;
    int lastSense = 0; // last LineTracker which saw line (1 for left, 2 for right)
    public void autonomousPeriodic()
    {

        shifter.set(true);
        try
        {
            setBreak(fLeft);
            setBreak(fRight);
            setBreak(bLeft);
            setBreak(bRight);
        }
        catch (Exception e)
        {

        }
         if (doneWithAuto)
         {
             return;
         }
         forkLeft = ds.getDigitalIn(1);//left
         pauseAtBegin = ds.getDigitalIn(2);
         stopAfterHang = ds.getDigitalIn(3);
         turnAfterHang = !stopAfterHang && ds.getDigitalIn(4);//This will only be true if stopAfterHang is false
         usingFork = ds.getDigitalIn(5);
         updateComp();
         updateDS();
         boolean leftValue = left.get();
         boolean middleValue = middle.get();
         boolean rightValue = right.get();
         //boolean leftValue2 = left2.get();
         //boolean middleValue2 = middle2.get();
         //boolean rightValue2 = right2.get();
         int height = (int)(ds.getDigitalIn(5)?0:1)+
                        (int)(ds.getDigitalIn(6)?0:2)+
                        (int)(ds.getDigitalIn(7)?0:4)+
                        (int)(ds.getDigitalIn(8)?0:8);
        double speed = 0.3;
       // System.out.println(rightValue + " " + middleValue + " " + leftValue);
        int lineState = (int)(rightValue?0:1)+
                        (int)(middleValue?0:2)+
                        (int)(leftValue?0:4);

        if(!armAtHeight)
        {
            setArmHeight(height);
            armAtHeight = true;
        }//raises arm for autonomous

        if (ds.getAnalogIn(1) > 0)
        {
            //dead reckoning
            straight(speed);
            if(closerThan(665))
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
        else if (!hasHangedTube)
        {
             moveWhileTracking(lineState, speed, autoState);
        }


        //System.out.println(ultraSonic.getRangeMM());

        if(closerThan(665))
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
    boolean KrakenIsWaiting = false;
    public void teleopPeriodic()
    {

         System.out.println(ultraSonic.getRangeMM());
        try{
        setCoast(fLeft); // set them to drive in coast mode (no sudden brakes)
        setCoast(fRight);
        setCoast(bLeft);
        setCoast(bRight);
        //setBreak(lowerArm);
        //setBreak(upperArm);
        }catch (Exception e) {}
        updateComp();
        updateGear();
        updateDS();

        if(controller.getRawButton(6))
        {
            KrakenIsWaiting = true;
        }
        else if(KrakenIsWaiting)
        {
            changeKraken();
            KrakenIsWaiting = false;
        }
        updateLowerArm();
        updateUpperArm();

        if(j2.getRawButton(10) && controller.getRawButton(2) && controller.getRawButton(8))
            minibot.set(false);
        
        if(!breaking())
        {
         setLefts(deadzone(-j1.getY()));
         setRights(deadzone(-j2.getY()));
        }
    }

    private void setLefts(double d)
    {
        try
        {
        fLeft.setX(d);
        bLeft.setX(d);

        } 
        catch (CANTimeoutException e)
        {
            DriverStationLCD lcd = DriverStationLCD.getInstance();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "CAN EXCEPTION");
            lcd.updateLCD();
        }
    }

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
        ds.setDigitalOut(5,  shifter.get());
    }

    private void setRights(double d)
    {
        try{
        fRight.setX(-d);
        bRight.setX(-d);
        } catch (CANTimeoutException e){
            e.printStackTrace();
            DriverStationLCD lcd = DriverStationLCD.getInstance();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "CAN EXCEPTION!!!");
            lcd.updateLCD();
        }
    }

    public void setCoast(CANJaguar jag) throws CANTimeoutException
    {//Sets the drive motors to coast mode
        try{jag.configNeutralMode(CANJaguar.NeutralMode.kCoast);} catch (Exception e) {e.printStackTrace();}
    }

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

     public void setBreak(CANJaguar jag) throws CANTimeoutException
    {//Sets the drive motors to brake mode
        try{jag.configNeutralMode(CANJaguar.NeutralMode.kBrake);} catch (Exception e) {e.printStackTrace();}
    }

    public double deadzone(double d)
    {//deadzone for input devices
        if (Math.abs(d) < .1) {
            return 0;
        }
        return d / Math.abs(d) * ((Math.abs(d) - .1) / .9);
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
        if(tinyDeadzone(-controller.getY()) == 0)
                break1.set(true);
        Sholder.set(tinyDeadzone(-controller.getY()));
    }

    public void updateUpperArm()
    {
       if(tinyDeadzone(-controller.getRawAxis(5)) == 0)
                break2.set(true);
        Elbow.set(-tinyDeadzone(controller.getRawAxis(5)));

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
                speed = .4;
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

    public void hangTube()
    {

         Kraken.set(true);
                        try
                        {
                            Thread.sleep(500); //And after two seconds...
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
          switch(height){
                default:
                        break;
                case 5:
                    //low middle
                    //THERE NEEDS TO BE A LOOP HERE!!!
                    break;
                case 9:
                    //low high
                     //THERE NEEDS TO BE A LOOP HERE!!!
                    break;
                case 2:
                    //high low
                     //THERE NEEDS TO BE A LOOP HERE!!!
                    break;
                case 4:
                    //high middle
                     //THERE NEEDS TO BE A LOOP HERE!!!
                    break;
                case 8:
                    //high high
                     //THERE NEEDS TO BE A LOOP HERE!!!
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

