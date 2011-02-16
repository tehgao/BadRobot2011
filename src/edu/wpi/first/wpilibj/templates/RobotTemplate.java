/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
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

    Joystick j1 = new Joystick(1);
    Joystick j2 = new Joystick(2);
    Joystick controller = new Joystick(3);
    CANJaguar fLeft, fRight, bLeft, bRight; //lowerArm, upperArm; //motors
    DigitalOutput output; // for ultrasonic
    DigitalInput input;
    Ultrasonic ultraSonic;
    AxisCamera cam; // camera
    Timer timer = new Timer(); // timer
    DigitalInput left; // for LineTracker
    DigitalInput middle;
    DigitalInput right;
    DriverStation ds;
    Compressor air;
    Solenoid shifter;
    boolean forkLeft;
    boolean pauseAtBegin; //Will the robot pause at the beginning of autonomous before moving?
    boolean stopAfterHang; //Will the robot stop after it hangs a ubertube?
    boolean turnAfterHang; //Will the robot turn as it's backing away, or go straight back? (Assuming that stopAfterHang is false)
    boolean hasHangedTube; // Has the robot hanged its ubertube (or at least attempted to)?
    boolean hasAlreadyPaused; //Has the robot already paused at the beginning? (Assuming that pauseAtBegin is true)
    boolean doneWithAuto; //Has the robot done what it needs to in auto mode?

    public void robotInit()
    {
            try
            {
                fLeft = new CANJaguar(10); // motors for wheels with CAN ports as arguements
                fRight = new CANJaguar(4);
                bLeft = new CANJaguar(9);
                bRight = new CANJaguar(7);
               // lowerArm = new CANJaguar(5);
              //  upperArm = new CANJaguar(8);

               // setCoast(fLeft); // set them to drive in coast mode (no sudden brakes)
               // setCoast(fRight);
               // setCoast(bLeft);
               // setCoast(bRight);

                left = new DigitalInput(3); // for LineTracker
                middle = new DigitalInput(2);
                right = new DigitalInput(14);

                output = new DigitalOutput(10); // ultrasonic output
                input = new DigitalInput(8); //ultrasonic input
                ultraSonic  = new Ultrasonic(output, input, Ultrasonic.Unit.kMillimeter); //initialize ultrasonic
                ultraSonic.setEnabled(true);
                ultraSonic.setAutomaticMode(true);

                air = new Compressor(1,1);
                shifter = new Solenoid(8,1);
                shifter.set(false);

                ds = DriverStation.getInstance();
                hasHangedTube = false;
                hasAlreadyPaused = false;
                doneWithAuto = false;

            } catch (Exception e) { e.printStackTrace(); }
        timer.delay(1);
    }

    /**
     * This function is called periodically during autonomous
     */

    boolean atFork = false; // if robot has arrived at fork
    int lastSense = 0; // last LineTracker which saw line (1 for left, 2 for right)
    public void autonomousPeriodic()
    {


        try{
         setBreak(fLeft);
         setBreak(fRight);
         setBreak(bLeft);
         setBreak(bRight);
         }catch (Exception e) {}
         if (doneWithAuto)
         {
             return;
         }
         forkLeft =  ds.getDigitalIn(1);//left
         pauseAtBegin = ds.getDigitalIn(2);
         stopAfterHang = ds.getDigitalIn(3);
         turnAfterHang = !stopAfterHang && ds.getDigitalIn(4);//This will only be true if stopAfterHang is false
         updateComp();
         boolean leftValue = left.get();
         boolean middleValue = middle.get();
         boolean rightValue = right.get();
        //System.out.print("Autonomous Start");
        double speed = 0.3;
        int lineState = (int)(rightValue?1:0)+
                        (int)(middleValue?2:0)+
                        (int)(leftValue?4:0);

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

         if(closerThan(500))
        {
            straight(0); //Stop

            //Here I'm guessing we'd have the hangTube() method

            hasHangedTube = true;
            if (stopAfterHang) //If the robot is supposed to stay put after it hangs a tube
                doneWithAuto = true;
            return;
        }

        if (pauseAtBegin && !hasAlreadyPaused) //If the robot should pause at the beginning and it hasn't already paused...
        {
            try
            {
                Thread.sleep(3000); //Pause for 3 seconds
            }
            catch (Exception e)
            {
                e.printStackTrace();
            }

            hasAlreadyPaused = true; //The robot has now paused
        }
        moveWhileTracking(lineState, speed);

    }
  
    public void teleopPeriodic()
    {
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


        setLefts(deadzone(-j1.getY()));
        setRights(deadzone(-j2.getY()));
       // updateLowerArm();
       // updateUpperArm();
    }

    private void setLefts(double d)
    {
        try{
        fLeft.setX(d);
        bLeft.setX(d);
        } catch (CANTimeoutException e){
            DriverStationLCD lcd = DriverStationLCD.getInstance();
            lcd.println(DriverStationLCD.Line.kMain6, 1, "CAN EXPTION");
            lcd.updateLCD();
        }
    }

    private void updateDS()
    {
        ds.setDigitalOut(1, forkLeft);
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
            lcd.println(DriverStationLCD.Line.kMain6, 1, "CAN EXcEPTION!!!");
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
        /** if(j1.getTrigger() || j2.getTrigger())
            switchStateShift = true;
        else if(switchStateShift)
        {
            shifter.set(!shifter.get());
            switchStateShift = false;
        }**/
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
        if (Math.abs(d) < .05) {
            return 0;
        }
        return d / Math.abs(d) * ((Math.abs(d) - .05) / .95);
    }
    //comment
   

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
   /* public void updateLowerArm()
    {//state machine for the lower arm
        try{
            lowerArm.setX(deadzone(controller.getZ()));
         } catch (CANTimeoutException e){
                DriverStationLCD lcd = DriverStationLCD.getInstance();
                lcd.println(DriverStationLCD.Line.kMain6, 1, "Arm is failing");
                lcd.updateLCD();
            }

    }

    public void updateUpperArm()
    {
        if(controller.getRawButton(6))
        {
            System.out.println("Upper arm: .5");
            try{
            upperArm.setX(0.5);
             } catch (CANTimeoutException e){
                DriverStationLCD lcd = DriverStationLCD.getInstance();
                lcd.println(DriverStationLCD.Line.kMain6, 1, "Arm is failing");
                lcd.updateLCD();
            }
        }
        else if (controller.getRawButton(5))
        {
            System.out.println("Upper arm: -.5");
            try{
            upperArm.setX(-0.35);
             } catch (CANTimeoutException e){
                DriverStationLCD lcd = DriverStationLCD.getInstance();
                lcd.println(DriverStationLCD.Line.kMain6, 1, "Arm is failing");
                lcd.updateLCD();
            }
        }
        else
        {
            try{
            upperArm.setX(0.0);
             } catch (CANTimeoutException e){
                DriverStationLCD lcd = DriverStationLCD.getInstance();
                lcd.println(DriverStationLCD.Line.kMain6, 1, "Arm is failing");
                lcd.updateLCD();
            }
        }
        //feedMe.feed();
    }
    */
    public void moveWhileTracking(int lineState, double speed)
    {
      switch (lineState)
        {
            case 0: //No sensors see the line
                System.out.println("Lost the line: " + lastSense);
                speed = .25;
                 if (lastSense == 1) // left is last seen, go left
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
                    setLefts(0.2); // CAUTION!  Go Slowly!
                    setRights(0.2);
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
                }
                else
                {
                    hardRight(speed);
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
        }

    }

}
