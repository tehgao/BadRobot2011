
package code.badrobot.tracking;

import edu.wpi.first.wpilibj.image.Image;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.MonoImage;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.CurveOptions;
import edu.wpi.first.wpilibj.image.EllipseDescriptor;
import edu.wpi.first.wpilibj.image.EllipseMatch;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.RegionOfInterest;
import edu.wpi.first.wpilibj.image.ShapeDetectionOptions;

/**
 *
 * @author Benjamin Schroeder
 */


public class Camera implements LocationDevice
{

    public Location getLocation()
    {
        Location loc = new Location();
        return loc;
    }

    private AxisCamera cam;
    private ColorImage img;
    private EllipseDescriptor ellipseDescriptor;
    EllipseMatch[] ellipseMatches;
    MonoImage monoImage;


    public Camera()
    {
	this(AxisCamera.getInstance());
    }

    public Camera(AxisCamera c)
    {
	cam = c;
	init();
    }

    public void init()
    {
	cam.writeCompression(0);
	cam.writeBrightness(10);
	cam.writeResolution(AxisCamera.ResolutionT.k160x120);

        //double minMajorRadius =
        //ellipseDescriptor = new EllipseDescriptor ()
        // 11.5 min radius & 16.5 max radius
    }

    public Image getImage()
    {
        cam.freshImage();
	try{img= cam.getImage(); return img;}catch (Exception e){};
	return null;
    }


    public boolean seeCircles(ColorImage img)
    {
        //Converts the ColorImage into a MonoImage to detect ellipses or throw exception
        try {
        monoImage = img.getLuminancePlane();
        } catch(NIVisionException e) {};

        //Initializes the ellipse desciptor, detects the ellipses and sets them into an array called ellipseMatches
        try{
        ellipseDescriptor = new EllipseDescriptor(20,150,20,150); ellipseMatches = monoImage.detectEllipses(ellipseDescriptor);
        }catch (NIVisionException e){};

        //If array is not empty, return that there are possible targets otherwise, false.
        if (ellipseMatches.length != 0)
        {
            return true;
        }
        return false;
    }

    public void imageAnalysis()
    {
        for(int i = 0; i<ellipseMatches.length; i++)
        {
            double xPos;
            double yPos;

            double committmajorAxisValue = ellipseMatches[0].m_majorRadius;
            double committminorAxisValue = ellipseMatches[0].m_minorRadius;
            double committpurity = ellipseMatches[0].m_score;
            double committisItACircle = 1 - (committmajorAxisValue/committminorAxisValue);

            double tempmajorAxisValue = ellipseMatches[i].m_majorRadius;
            double tempminorAxisValue = ellipseMatches[i].m_minorRadius;
            double temppurity = ellipseMatches[i].m_score;
            double tempisItACircle = 1 - (tempmajorAxisValue/tempminorAxisValue);
            
            if (tempisItACircle<committisItACircle && temppurity>committpurity)
            {
                committmajorAxisValue = ellipseMatches[i].m_majorRadius;
                committminorAxisValue = ellipseMatches[i].m_minorRadius;
                committpurity = ellipseMatches[i].m_score;
                committisItACircle = 1 - (committmajorAxisValue/committminorAxisValue);
                xPos = ellipseMatches[i].m_xPos;
                yPos = ellipseMatches[i].m_yPos;

            }
        }

        /* code here to direct what the robot should do if it sees a circle*/
    }


}
