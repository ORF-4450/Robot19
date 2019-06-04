package Team4450.Robot19;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import Team4450.Lib.Util;
import Team4450.Robot19.GripPipeline;

public class Vision implements IVision
{
	private Robot 			robot;
	private GripPipeline	pipeline = new GripPipeline();
	private Rect			targetRectangle1 = null, targetRectangle2 = null;
	private int				centerX1 = 0, centerX2 = 0, centerY1 = 0, centerY2 = 0, imageCount;
	private int				xResolution = 320, yResolution = 240;
	private boolean			targetFound, saveImages;

	// This variable and method make sure this class is a singleton.
	
	public static Vision vision = null;
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @param robot Reference to the Robot class instance.
	* @return Reference to single shared instance of this class.
	*/
	public static Vision getInstance(Robot robot) 
	{
		if (vision == null) vision = new Vision(robot);
		
		return vision;
	}
	
	// Private constructor prevents multiple instances from being created.
	
	private Vision(Robot robot) 
	{
		this.robot = robot;
		
		Util.consoleLog("Vision created!");
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		vision =  null;
	}
	
	/**
	 * Save source and hsl filtered images to RoboRio disk. If you enable this you must
	 * delete any existing saved image files before each run.
	 * @param save True to save images.
	 */
	public void saveImages(boolean save)
	{
		saveImages = save;
	}
	
	/**
	 * Trigger the image processing pipeline to run one time with current image
	 * available internally. Only used if the pipeline does not run
	 * automatically.
	 */
	public void processImage()
	{
		
	}
	
	/**
	 * Trigger the image processing pipeline to run one time with the supplied
	 * image. Only used if the pipeline does not run automatically.
	 * @param image
	 */
	public void processImage(Mat image)
	{
		if (saveImages) Imgcodecs.imwrite(String.format("/home/lvuser/image%d.jpg", imageCount), image);
		
		pipeline.process(image);

		if (saveImages)
		{
			Imgcodecs.imwrite(String.format("/home/lvuser/imagehsl%d.jpg", imageCount), pipeline.hslThresholdOutput());

			imageCount++;
		}
		
		targetRectangle1 = targetRectangle2 = null;
		
		if (pipeline.filterContoursOutput().size() > 1)
		{
			// When more than 2 contours, filter to 2 by removing contours too small.
//			while (pipeline.filterContoursOutput().size() > 2)
//			{
//				boolean removed = false;
//				
//				for (int i = 0; i < pipeline.filterContoursOutput().size(); i++)
//				{
//					Rect rectangle  = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
//					
//					if (rectangle.height * rectangle.width < 300)
//					{
//						pipeline.filterContoursOutput().remove(i);
//						removed = true;
//						break;
//					}
//				}
//				
//				if (!removed) break;
//			}
			
			targetRectangle1 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
			targetRectangle2 = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
		}
		
		if (targetRectangle1 != null && targetRectangle2 != null)
		{
			centerX1 = targetRectangle1.x + targetRectangle1.width / 2;
			centerX2 = targetRectangle2.x + targetRectangle2.width / 2;

			centerY1 = targetRectangle1.y + targetRectangle1.height / 2;
			centerY2 = targetRectangle2.y + targetRectangle2.height / 2;

			Util.consoleLog("x1=%d y1=%d c=%d h=%d w=%d cnt=%d", targetRectangle1.x, targetRectangle1.y, centerX1, targetRectangle1.height,
			         targetRectangle1.width, pipeline.filterContoursOutput().size());

			Util.consoleLog("x2=%d y2=%d c=%d h=%d w=%d cnt=%d", targetRectangle2.x, targetRectangle2.y, centerX2, targetRectangle2.height,
			        targetRectangle2.width, pipeline.filterContoursOutput().size());
			
			targetFound = true;
		}
		else
		{
			Util.consoleLog("no targets found contours=%d", pipeline.filterContoursOutput().size());	//  image=%d", imageCount);
			targetFound = false;
		}
	}
	
	/**
	 * Indicates if vision system has a target in the camera field of vision as of
	 * the last run of the image processing pipeline.
	 * @return True if target visible, false if not.
	 */
	public boolean targetVisible()
	{
		return targetFound;
	}
	
	/**
	 * Returns an array of target rectangles derived from the contours returned
	 * by the last run of the image processing pipeline representing the targets
	 * in the field of vision of camera. Not valid if targetVisible is not true.
	 * @return An array of target rectangles or null.
	 */
	public ArrayList<Rect> getTargetRectangles()
	{
		if (!targetFound) return null;
		
		ArrayList<Rect> rects = new ArrayList<Rect>();
		
		rects.add(targetRectangle1);
		rects.add(targetRectangle2);
		
		return rects;
	}
	
	/**
	 * Returns the contours returned by the last run of the image processing pipeline representing
	 * the targets in the field of vision of camera. Not valid if targetVisible is
	 * not true.
	 * @return An array of target contours or null.
	 */
	public ArrayList<MatOfPoint> getContours()
	{
		if (!targetFound) return null;
		
		return pipeline.filterContoursOutput();
	}
	
	/**
	 * Returns the distance value to the target(s). The distance is typically the number of
	 * pixels separating two target centers in the camera field of vision.
	 * @return The distance to the targets.
	 */
	public double getDistance()
	{
		return Math.abs(centerX1 - centerX2);
	}
	/**
	 * Return target center offset X axis.
	 * @return The X axis offset from left edge.
	 */
	public int centerX()
	{
		int center = 0;
		
		if (centerX1 > centerX2)
			center = (centerX1 - centerX2) / 2 + centerX2;
		else
			center = (centerX2 - centerX1) / 2 + centerX1;
		
		return center;
	}
	
	/**
	 * Return target center offset Y axis.
	 * @return The Y axis offset from top edge.
	 */
	public int centerY()
	{
		int center = 0;
		
		if (centerY1 > centerY2)
			center = (centerY1 - centerY2) / 2 + centerY2;
		else
			center = (centerY2 - centerY1) / 2 + centerY1;
		
		return center;
	}
	
	/**
	 * Return target center X offset from field of vision center.
	 * @return Target center X axis offset from field of vision center, - is left of
	 * center, + is right of center.
	 */
	public int offsetX()
	{
		return centerX() - xResolution / 2;
	}
	
	/**
	 * Return target center Y offset from field of vision center.
	 * @return Target center Y axis offset from field of vision center, - is below 
	 * center, + is above center.
	 */
	public int offsetY()
	{
		return yResolution / 2 - centerY();
	}

	/**
	 * Set camera resolution. Defaults to 320x240.
	 * @param x X axis resolution.
	 * @param y Y Axis resolution.
	 */
	public void setResolution( int x, int y )
	{
		xResolution = x;
		yResolution = y;
	}
}
