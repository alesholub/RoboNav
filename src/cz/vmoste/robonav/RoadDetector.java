package cz.vmoste.robonav; 

import java.util.ArrayList;
import java.util.Iterator;
//import java.util.Iterator;
import java.util.List;
import java.util.Vector;

import org.opencv.core.Core;
import org.opencv.core.CvType;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class RoadDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private int mDirection = 0;
    private int mTopDirection = 0;
    private int mTopHeight = 0;
	private Point topPoint = new Point(0,0);

    // Cache
    Mat mGreyMat = new Mat();
    Mat mHierarchy = new Mat();
    Mat mResultMat = new Mat();

    int edgeThresh = 1;
    int lowThreshold = 50;
    int max_lowThreshold = 100;
    int mLevel = 140;
    int mLevel2 = 240;
    int mSearchMode = 1;

    private int limit1 = 10;
    private int limit2 = 25;

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
        limit1 = sLimit1;
        limit2 = sLimit2;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void process(Mat rgbaImage) {
    	int w = rgbaImage.width();
    	int h = rgbaImage.height();
    	int lin = h/4;
        int mod6 = mLevel % 6;
        Mat mZeroMat = new Mat(h, w, 0, new Scalar(255, CvType.CV_8UC1));
    	if (mSearchMode>=0) {
            Vector<Mat> channels = new Vector<Mat>();
            Imgproc.cvtColor(rgbaImage, mResultMat, Imgproc.COLOR_RGB2HSV_FULL);
            Core.split(mResultMat, channels);
            //channels.set(2,new Mat(h, w, 0, new Scalar(220))); //Set V
        	if (mod6!=2 && mod6!=5) {
        		// detect road by combining H and S channels
        		mResultMat = channels.get(0);
            	Imgproc.threshold(mResultMat, mResultMat, mLevel/2+20, 255, Imgproc.THRESH_BINARY);
            	Imgproc.threshold(channels.get(1), mGreyMat, 255-mLevel, 255, Imgproc.THRESH_BINARY_INV);
            	Core.add(mGreyMat, mResultMat, mGreyMat);
            	//channels.set(0,mGreyMat);
            	//channels.set(1,mGreyMat);
            	//channels.set(2,mGreyMat);
                //Core.merge(channels, rgbaImage);;
        	} else {
        		// detect road by S channel only
                mGreyMat = mZeroMat;
            	Core.subtract(mGreyMat, channels.get(1), mGreyMat);
        	}
    	}
    	//Imgproc.cvtColor(rgbaImage, mGreyMat, Imgproc.COLOR_RGB2GRAY );
    	//Imgproc.GaussianBlur(mResultMat, mResultMat, new org.opencv.core.Size(9,9), 0); // 31.3.2016
    	if (mod6<6) Imgproc.blur(mGreyMat, mGreyMat, new org.opencv.core.Size(w/100,w/100));
        Mat kernel1 = Mat.ones(w/100, w/100, CvType.CV_8UC1); // 30.03.2016
        if (mod6<3) Imgproc.dilate(mGreyMat, mGreyMat, kernel1); // 05.05.2016
        //Imgproc.erode(mResultMat, mResultMat, kernel1); // 30.03.2016
    	//if (mod4>1) Imgproc.equalizeHist(mGreyMat, mGreyMat);
    	//if (mod4>1) Core.normalize(mGreyMat, mGreyMat);
    	if (mOrientation==2) {
    		// landscape
        	//Core.line(mResultMat, new Point(0,lin/8), new Point(w,lin/8), new Scalar(0,0,0), lin/4);
        	Core.line(mGreyMat, new Point(0,h-lin/4), new Point(w,h-lin/4), new Scalar(0,0,0), lin/2);
        	Core.line(mGreyMat, new Point(lin/8,0), new Point(lin/8,h), new Scalar(0,0,0), lin/4);
        	Core.line(mGreyMat, new Point(w-lin/8,0), new Point(w-lin/8,h), new Scalar(0,0,0), lin/4);
        	Core.line(mGreyMat, new Point(w/6,0), new Point(0,h/2), new Scalar(0,0,0), lin);
        	Core.line(mGreyMat, new Point(w-w/6,0), new Point(w,h/2), new Scalar(0,0,0), lin);
    	} else {
    		// portrait
        	Core.line(mGreyMat, new Point(lin/2,0), new Point(lin/2,h), new Scalar(0,0,0), lin);
        	Core.line(mGreyMat, new Point(w-lin,0), new Point(w-lin,h), new Scalar(0,0,0), 2*lin);
        	Core.line(mGreyMat, new Point(0,lin), new Point(w/2-lin,0), new Scalar(0,0,0), lin);
        	Core.line(mGreyMat, new Point(0,h-lin), new Point(w/2-lin,h), new Scalar(0,0,0), lin);
    	}
    	//Imgproc.watershed(mResultMat, mResultMat);
    	if (mSearchMode<=9) {
    		//if (mod4>1) Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY+Imgproc.THRESH_OTSU);
    		//else Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY);
        	Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY+Imgproc.THRESH_OTSU);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY_INV);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY_INV+Imgproc.THRESH_OTSU);
        	
    	}

    	List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mGreyMat, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
		MatOfPoint mContour = null;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea) {
                maxArea = area;
                mContour = wrapper;
            }
        }
        mContours.clear();
        mContours.add(mContour);
        
        // compute direction to max contour moment (center)
        mDirection = 0;
    	double x = w/2;
    	double y = h/2; 
    	topPoint.x = w-2*lin-1;
    	topPoint.y = y;
    	if (mOrientation==2) {
    		topPoint.x = x; 
        	topPoint.y = h-lin-1;
    	}
    	double x0 = 0;
    	double y0 = 0; 
    	double x1 = 0;
    	double y1 = 0; 
    	if (mContour!=null) {
        	Moments mo = new Moments();
    		mo = Imgproc.moments(mContour);
    		x = (mo.get_m10() / mo.get_m00());
    		y = (mo.get_m01() / mo.get_m00());
    		if (mOrientation==2) mDirection = (int)Math.round(Math.toDegrees(Math.atan((x-(w/2))/(h-lin-y))));
    		else mDirection = (int)Math.round(Math.toDegrees(Math.atan((h/2-y)/(w-2*lin-x))));
        	//Imgproc.drawContours(rgbaImage, mContours, 0, new Scalar(255,255,255), Core.FILLED);
        	Imgproc.drawContours(rgbaImage, mContours, 0, new Scalar(255,0,0), 2);
        	MatOfInt hull = new MatOfInt();
        	Imgproc.convexHull(mContour, hull);
        	for(int i = 0; i < hull.size().height ; i++)
        	{
        	    int index = (int)hull.get(i, 0)[0];
        	    x1 = mContour.get(index, 0)[0];
        	    y1 = mContour.get(index, 0)[1];
        		if (mOrientation!=2) {
        			if (x1<topPoint.x) topPoint = new Point(x1, y1);
        			else if (x1==topPoint.x) topPoint = new Point(x1, (topPoint.y+y1)/2);
        		} else {
        			if (y1<topPoint.y) topPoint = new Point(x1, y1); 
        			else if (y1==topPoint.y) topPoint = new Point((topPoint.x+x1)/2, y1);
        		}
        		//Core.circle(rgbaImage, new Point(x1, y1), 5, new Scalar(255,49,0,255), 1);
        		if (i>0) Core.line(rgbaImage, new Point(x0, y0), new Point(x1, y1), new Scalar(0,0,255), 1);
        		x0 = x1; y0 = y1;
        	}
    		Core.circle(rgbaImage, topPoint, 6, new Scalar(255,0,0), -1);
    	}
    	// topPoint direction
		if (mOrientation==2) mTopDirection = (int)Math.round(Math.toDegrees(Math.atan((topPoint.x-(w/2))/(h-lin-topPoint.y))));
		else mTopDirection = (int)Math.round(Math.toDegrees(Math.atan((h/2-topPoint.y)/(w-2*lin-topPoint.x))));
    	// topPoint height
		if (mOrientation==2) mTopHeight = (int)Math.round((h-lin-topPoint.y)/lin);
		else mTopHeight = (int)Math.round((w-2*lin-topPoint.x)/lin);
    	//x = topPoint.x;
    	//y = topPoint.y;
		if (mOrientation==2) mDirection = (int)Math.round(Math.toDegrees(Math.atan((x-(w/2))/(h-lin-y))));
		else mDirection = (int)Math.round(Math.toDegrees(Math.atan((h/2-y)/(w-2*lin-x))));
    	Scalar mColor = new Scalar(0,255,0); // yellow
		if (mDirection<-limit1 || mDirection>limit1) mColor = new Scalar(0,0,255); // blue
		if (mDirection<-limit2 || mDirection>limit2) mColor = new Scalar(255,0,0); // red
		Core.circle(rgbaImage, new Point(x, y), 5, new Scalar(255,49,0,255));
		if (mOrientation==2) {
			// landscape
			Core.line(rgbaImage, new Point(x,y), new Point(w/2,h-lin), mColor, 3);
			Core.line(rgbaImage, new Point(topPoint.x,topPoint.y), new Point(w/2,h-lin), new Scalar(255,255,0), 1);
		} else {
			// portrait
			Core.line(rgbaImage, new Point(x,y), new Point(w-2*lin,h/2), mColor, 3);
			Core.line(rgbaImage, new Point(topPoint.x,topPoint.y), new Point(w-2*lin,h/2), new Scalar(255,255,0), 1);
		}
    	mResultMat = rgbaImage;
    }

    public Mat getResultMat() {
        return mResultMat;
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }

    public int getDirection() {
        return mDirection;
    }

    public int getTopDirection() {
        return mTopDirection;
    }

    public int getTopHeight() {
        return mTopHeight;
    }
}
