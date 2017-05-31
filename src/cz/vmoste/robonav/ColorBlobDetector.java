package cz.vmoste.robonav; 

import java.util.ArrayList;
//import java.util.Iterator;
import java.util.List;

import org.opencv.core.Core;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;

public class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    //private Scalar mHsvColor = new Scalar(0);
    // Minimum contour area in percent for contours filtering
    //private static double mMinContourArea = 0.7;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
    //private Scalar mColorRadius = new Scalar(65,90,90,0);
    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private int direction = 0;
    private int mTopDirection = 0;
    private int mTopHeight = 0;
	private Point mTopPoint = new Point(0,0);
	private Rect mBoundingRectangle = null;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();
    Mat mResultMat = new Mat();
    double minArea = 50;

    int mLevel = 140;
    int mLevel2 = 240;
    int mSearchMode = 1;

//    private int limit1 = 10;
//    private int limit2 = 25;

    private int mOrientation = 0;

    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setOrientation(int sOrientation) {
        mOrientation = sOrientation;
    }

    public void setSearchMode(int sMode) {
        mSearchMode = sMode;
    }

    public void setLevel(int sLevel) {
        mLevel = sLevel;
        mLevel2 = sLevel + 50;
        if (mLevel2>255) mLevel2 = 255;
    }

    public void setLimits(int sLimit1, int sLimit2) {
        //limit1 = sLimit1;
        //limit2 = sLimit2;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = (hsvColor.val[1] >= mColorRadius.val[1]) ? hsvColor.val[1] - mColorRadius.val[1] : 0;
        mUpperBound.val[1] = (hsvColor.val[1]+mColorRadius.val[1] <= 255) ? hsvColor.val[1] + mColorRadius.val[1] : 255;

        mLowerBound.val[2] = (hsvColor.val[2] >= mColorRadius.val[2]) ? hsvColor.val[2] - mColorRadius.val[2] : 0;
        mUpperBound.val[2] = (hsvColor.val[2]+mColorRadius.val[2] <= 255) ? hsvColor.val[2] + mColorRadius.val[2] : 255;

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
        
        //mHsvColor = hsvColor;

    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        //mMinContourArea = area;
    }

    public void setMinArea(double area) {
        minArea = area;
    }

    public void process(Mat mRgba) {
    	// detect highest area with defined color
    	// return contour of this area
    	// return direction to the center of the area
    	// return central point of the area
    	// return bounding rectangle of the area
        
    	Imgproc.pyrDown(mRgba, mPyrDownMat);
        //Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find highest area
		MatOfPoint mContour0 = null;
		//MatOfPoint mContour1 = null;
    	int w = mRgba.width();
    	int h = mRgba.height();

        mBoundingRectangle = new Rect(w/2,h/2,1,1);
        //double maxArea = 0;
        //maxArea = minArea;
        //minArea = w/10;
        int maxVal = 0;
        int maxValIdx = 0;
        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
        {
            //double contourArea = Imgproc.contourArea(contours.get(contourIdx));
            Rect tmpRect = Imgproc.boundingRect(contours.get(contourIdx));
            int tmpVal = tmpRect.height;
            if (maxVal < tmpVal)
            {
                maxVal = tmpVal;
                maxValIdx = contourIdx;
                //mBoundingRectangle = tmpRect.clone();
            }
        }
        if (contours.size()>0) mContour0 = (MatOfPoint)contours.get(maxValIdx);

        // only two biggest contours must left
        //contours.clear();
        //if (mContour0!=null) contours.add(mContour0);
        //if (mContour1!=null) contours.add(mContour1);
        mContours.clear();
        if (mContour0!=null) {
    		//Core.multiply(mContour0, new Scalar(4,4), mContour0);
    		Core.multiply(mContour0, new Scalar(2,2), mContour0);
    		mContours.add(mContour0);
        }

        Scalar CONTOUR_COLOR = new Scalar(255,0,0,255);
		if (mSearchMode==2) {
    	  Imgproc.drawContours(mRgba, mContours, -1, CONTOUR_COLOR, 2);
		}

    	int x = w/2;
    	int y = h/2;
		double x3 = x;
		double y3 = y;
		mTopPoint = new Point(x3, y3);
		direction = 0;
    	if (mContours.size()>0) {
    		// set direction by position of first two contours
    		MatOfPoint contour = mContours.get(0);
    		mBoundingRectangle = Imgproc.boundingRect(contour);
        	//mTxt1 = ""+contour.cols()+" / "+mBoundingRectangle.toString();
        	//Core.putText(mRgba, ""+mTxt1, new Point(w/4,h/3), 1, 2, new Scalar(255,255,50), 2);
        	//Core.rectangle(mRgba, mBoundingRectangle.tl(), mBoundingRectangle.br(),  new Scalar(0,255,0), 3);
//    		mo = Imgproc.moments(contour);
//    		x = (int) (mo.get_m10() / mo.get_m00());
//    		y = (int) (mo.get_m01() / mo.get_m00());
    		//if (mBoundingRectangle.height<(h/200) || (mBoundingRectangle.x<=(2*corner) && mBoundingRectangle.y<=(2*corner))) {
      		if (mBoundingRectangle.height<(h/100)) {
    			x = w/2;
    			y = h/2;
    	        mBoundingRectangle = new Rect(x,y,1,1);
    		} else {
    			x = (int) (mBoundingRectangle.x + mBoundingRectangle.width/2);
    			y = (int) (mBoundingRectangle.y + mBoundingRectangle.height/2);
    		}
    		//Core.circle(mRgba, new Point(x, y), 40, new Scalar(255,255,0), -1);
			x3 = x;
			y3 = y;
    		mTopPoint = new Point(x3, y3);
        	//mTxt1 = ""+mTopPoint.toString();
        	//Core.putText(mRgba, ""+mTxt1, new Point(w/4,h/3), 1, 2, new Scalar(255,255,50), 2);
//			Core.circle(mRgba, mTopPoint, 6, new Scalar(255,49,0,255),2);
//			Point endPoint = new Point(w/2,h);
			if (mOrientation==2) {
				// landscape
        		//direction = (int)Math.round(Math.toDegrees(Math.atan((x3-(w/2))/(h-y3))));
				direction = 200*(x-w/2)/w + 100*mBoundingRectangle.height/h;
			} else {
				// portrait
        		//direction = (int)Math.round(Math.toDegrees(Math.atan((h/2-y3)/(w-x3))));
//        		endPoint = new Point(w,h/2);
				direction = 200*(h/2-y)/h + 100*mBoundingRectangle.width/w;
			}
        }
		//mResultMat = mRgba;
    }

    public Mat getResultMat() {
        return mResultMat;
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }

    public int getDirection() {
        return direction;
    }

    public int getTopDirection() {
        return mTopDirection;
    }

    public int getTopHeight() {
        return mTopHeight;
    }

    public Point getTopPoint() {
        return mTopPoint;
    }

    public Rect getBoundingRectangle() {
        return mBoundingRectangle;
    }

}
