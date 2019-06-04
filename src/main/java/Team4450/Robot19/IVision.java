package Team4450.Robot19;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;

/**
 * Standard interface to various vision technologies.
 */
public interface IVision
{
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @param robot Reference to the Robot class instance.
	* @return Reference to single shared instance of this class.
	*/
	public static Object getInstance(Robot robot)
	{
		return null;
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose();
	
	/**
	 * Save source and hsl filtered images to RoboRio disk. If you enable this you must
	 * delete any existing saved image files before each run.
	 * @param save True to save images.
	 */
	public void saveImages(boolean save);
	
	/**
	 * Trigger the image processing pipeline to run one time with current image
	 * available internally. Only used if the pipeline does not run
	 * automatically.
	 */
	public void processImage();
	
	/**
	 * Trigger the image processing pipeline to run one time with the supplied
	 * image. Only used if the pipeline does not run automatically.
	 * @param image
	 */
	public void processImage(Mat image);
	
	/**
	 * Indicates if vision system has a target in the camera field of vision as of
	 * the last run of the image processing pipeline.
	 * @return True if target visible, false if not.
	 */
	public boolean targetVisible();
	
	/**
	 * Returns an array of target rectangles derived from the contours returned
	 * by the last run of the image processing pipeline representing the targets
	 * in the field of vision of camera. Not valid if targetVisible is not true.
	 * @return An array of target rectangles or null.
	 */
	public ArrayList<Rect> getTargetRectangles();
	
	/**
	 * Returns the contours returned by the last run of the image processing pipeline representing
	 * the targets in the field of vision of camera. Not valid if targetVisible is
	 * not true.
	 * @return An array of target contours or null.
	 */
	public ArrayList<MatOfPoint> getContours();
	
	/**
	 * Returns the distance value to the target(s). The distance is typically the number of
	 * pixels separating two target centers in the camera field of vision.
	 * @return The distance to the targets.
	 */
	public double getDistance();

	/**
	 * Return target center offset X axis.
	 * @return The X axis offset from left edge.
	 */
	public int centerX();
	
	/**
	 * Return target center offset Y axis.
	 * @return The Y axis offset from top edge.
	 */
	public int centerY();
	
	/**
	 * Return target center X offset from field of vision center.
	 * @return Target center X axis offset from field of vision center, - is left of
	 * center, + is right of center.
	 */
	public int offsetX();
	
	/**
	 * Return target center Y offset from field of vision center.
	 * @return Target center Y axis offset from field of vision center, - is below 
	 * center, + is above center.
	 */
	public int offsetY();
	
	/**
	 * Set camera resolution. Defaults to 320x240.
	 * @param x X axis resolution.
	 * @param y Y Axis resolution.
	 */
	public void setResolution(int x, int y);
}
