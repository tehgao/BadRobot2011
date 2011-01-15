package code.badrobot.tracking;

import code.badrobot.tracking.Target;
import code.badrobot.tracking.Camera;
import edu.wpi.first.wpilibj.image.Image;


/**
 *
 * @author Benjamin Schroeder
 */


public class Tracker
{
    private Camera myCam;
    private Target myTarg;

    public Tracker(Camera c)
    {
	myCam=c;
    }

    private Image getImage()
    {
	return myCam.getImage();
    }

    public void target()
    {
	
    }


}
