/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
/*

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author emma
 */
/*
public class CheckLimit extends Thread {
    DigitalInput limitSwitch[];
    Victor lowerArm;
    Victor upperArm;

    public CheckLimit(DigitalInput lswitch[], Victor lArm, Victor uArm) {
        limitSwitch = lswitch;
        lowerArm = lArm;
        upperArm = uArm;
    }

    public void run()
    {
        while (true)
        {
            for(int k=0; k<limitSwitch.length; k++) {
                if(limitSwitch[k].get()) {
                    lowerArm.set(0);
                    upperArm.set(0);
                }
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
    }
}
*/