package cz.vmoste.robonav; 

import java.util.ArrayList;
import java.util.Iterator;
//import java.util.Iterator;
import java.util.List;
import java.util.Vector;

import org.opencv.core.Core;
//import org.opencv.core.CvType;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
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
	private Point centPoint = new Point(0,0);
    private int mLeftOK = 1;
    private int mRightOK = 1;
    private int mCenterOK = 1;
	int w = 1;
	int h = 1;

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

    //private int limit1 = 10;
    //private int limit2 = 25;

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

    public static Moments contourMoments (MatOfPoint contour) {
        Moments m = new Moments();
        int lpt = contour.checkVector(2);
        //boolean is_float = true;//(contour.depth() == CvType.CV_32F);
        Point[] ptsi = contour.toArray();

        //CV_Assert( contour.depth() == CV_32S || contour.depth() == CV_32F );

        if( lpt == 0 )
                return m;

        double a00 = 0, a10 = 0, a01 = 0, a20 = 0, a11 = 0, a02 = 0, a30 = 0, a21 = 0, a12 = 0, a03 = 0;
        double xi, yi, xi2, yi2, xi_1, yi_1, xi_12, yi_12, dxy, xii_1, yii_1;


        {
                xi_1 = ptsi[lpt-1].x;
                yi_1 = ptsi[lpt-1].y;
        }

        xi_12 = xi_1 * xi_1;
        yi_12 = yi_1 * yi_1;

        for( int i = 0; i < lpt; i++ )
        {

                {
                        xi = ptsi[i].x;
                        yi = ptsi[i].y;
                }

                xi2 = xi * xi;
                yi2 = yi * yi;
                dxy = xi_1 * yi - xi * yi_1;
                xii_1 = xi_1 + xi;
                yii_1 = yi_1 + yi;

                a00 += dxy;
                a10 += dxy * xii_1;
                a01 += dxy * yii_1;
                a20 += dxy * (xi_1 * xii_1 + xi2);
                a11 += dxy * (xi_1 * (yii_1 + yi_1) + xi * (yii_1 + yi));
                a02 += dxy * (yi_1 * yii_1 + yi2);
                a30 += dxy * xii_1 * (xi_12 + xi2);
                a03 += dxy * yii_1 * (yi_12 + yi2);
                a21 += dxy * (xi_12 * (3 * yi_1 + yi) + 2 * xi * xi_1 * yii_1 +
                        xi2 * (yi_1 + 3 * yi));
                a12 += dxy * (yi_12 * (3 * xi_1 + xi) + 2 * yi * yi_1 * xii_1 +
                        yi2 * (xi_1 + 3 * xi));
                xi_1 = xi;
                yi_1 = yi;
                xi_12 = xi2;
                yi_12 = yi2;
        }
        float FLT_EPSILON = 1.19209e-07f;
        if( Math.abs(a00) > FLT_EPSILON )
        {
                double db1_2, db1_6, db1_12, db1_24, db1_20, db1_60;

                if( a00 > 0 )
                {
                        db1_2 = 0.5;
                        db1_6 = 0.16666666666666666666666666666667;
                        db1_12 = 0.083333333333333333333333333333333;
                        db1_24 = 0.041666666666666666666666666666667;
                        db1_20 = 0.05;
                        db1_60 = 0.016666666666666666666666666666667;
                }
                else
                {
                        db1_2 = -0.5;
                        db1_6 = -0.16666666666666666666666666666667;
                        db1_12 = -0.083333333333333333333333333333333;
                        db1_24 = -0.041666666666666666666666666666667;
                        db1_20 = -0.05;
                        db1_60 = -0.016666666666666666666666666666667;
                }

                // spatial moments
                m.m00 = a00 * db1_2;
                m.m10 = a10 * db1_6;
                m.m01 = a01 * db1_6;
                m.m20 = a20 * db1_12;
                m.m11 = a11 * db1_24;
                m.m02 = a02 * db1_12;
                m.m30 = a30 * db1_20;
                m.m21 = a21 * db1_60;
                m.m12 = a12 * db1_60;
                m.m03 = a03 * db1_20;

              // m.completeState();
        }
        return m;
    }
    
    public void process(Mat mRgba) {
    	w = mRgba.width();
    	h = mRgba.height();
        int ww = w/4;
        int hh = h/4;
    	int lin = hh/4;
		centPoint = new Point(ww,hh);
        int mod6 = mLevel % 6;
        //Mat mZeroMat = new Mat(hh, ww, 0, new Scalar(255, CvType.CV_8UC1));
        //Imgproc.pyrDown(mRgba, mResultMat);
		Imgproc.resize(mRgba, mResultMat, new Size(ww,hh));
        //mZeroMat.copyTo(mGreyMat);
        //Imgproc.pyrDown(mResultMat, mResultMat);
//        if (mSearchMode>=0) return;
    	if (mSearchMode>=0) {
    		//mRgba.copyTo(mResultMat);
    		//Imgproc.resize(mRgba, mResultMat, new Size(w/2,h/2));
            Vector<Mat> channels = new Vector<Mat>();
            //Imgproc.cvtColor(mRgba, mResultMat, Imgproc.COLOR_RGB2HSV_FULL);
            Imgproc.cvtColor(mResultMat, mResultMat, Imgproc.COLOR_RGB2HSV_FULL);
            Core.split(mResultMat, channels);
            //channels.set(2,new Mat(h, w, 0, new Scalar(220))); //Set V
        	if (mod6!=2 && mod6!=3 && mod6!=5) {
        		// detect road by combining H and S channels
        		mResultMat = channels.get(0);
            	Imgproc.threshold(mResultMat, mResultMat, mLevel/2+20, 255, Imgproc.THRESH_BINARY);
            	Imgproc.threshold(channels.get(1), mGreyMat, 255-mLevel, 255, Imgproc.THRESH_BINARY_INV);
            	//Core.add(mGreyMat, mResultMat, mGreyMat);
            	Core.bitwise_or(mGreyMat, mResultMat, mGreyMat);
        	} else {
        		// detect road by S channel only
                //mGreyMat = mZeroMat;
            	//Core.subtract(mGreyMat, channels.get(1), mGreyMat);
            	//Core.subtract(mGreyMat, new Mat(), mGreyMat);
        		Core.bitwise_not(channels.get(1),mGreyMat);
        	}
    	}
    	//mResultMat = mRgba;
    	//Imgproc.cvtColor(rgbaImage, mGreyMat, Imgproc.COLOR_RGB2GRAY );
    	//Imgproc.GaussianBlur(mResultMat, mResultMat, new org.opencv.core.Size(9,9), 0); // 31.3.2016
    	if (mod6<6) Imgproc.blur(mGreyMat, mGreyMat, new org.opencv.core.Size(w/50,w/50));
        //Mat kernel1 = Mat.ones(w/100, w/100, CvType.CV_8UC1); // 30.03.2016
        //if (mod6<3) Imgproc.dilate(mGreyMat, mGreyMat, kernel1); // 05.05.2016
        //Imgproc.erode(mGreyMat, mGreyMat, kernel1); // 2.5.2017
    	//if (mod4>1) Imgproc.equalizeHist(mGreyMat, mGreyMat);
    	//if (mod4>1) Core.normalize(mGreyMat, mGreyMat);
        int linx = lin/2;
    	if (mOrientation==2) {
    		// landscape
        	//Core.line(mResultMat, new Point(0,linx/8), new Point(w,linx/8), new Scalar(0,0,0), linx/4);
    		Imgproc.line(mGreyMat, new Point(0,hh-linx/4), new Point(ww,hh-linx/4), new Scalar(0,0,0), linx/2);
    		Imgproc.line(mGreyMat, new Point(linx/8,0), new Point(linx/8,hh), new Scalar(0,0,0), linx/4);
    		Imgproc.line(mGreyMat, new Point(ww-linx/8,0), new Point(ww-linx/8,hh), new Scalar(0,0,0), linx/4);
    		Imgproc.line(mGreyMat, new Point(ww/6,0), new Point(0,hh/2), new Scalar(0,0,0), linx);
    		Imgproc.line(mGreyMat, new Point(ww-ww/6,0), new Point(ww,hh/2), new Scalar(0,0,0), linx);
    	} else {
    		// portrait
    		Imgproc.line(mGreyMat, new Point(linx/2,0), new Point(linx/2,hh), new Scalar(0,0,0), linx);
    		Imgproc.line(mGreyMat, new Point(ww-linx,0), new Point(ww-linx,hh), new Scalar(0,0,0), 2*linx);
    		Imgproc.line(mGreyMat, new Point(0,linx), new Point(ww/2-linx,0), new Scalar(0,0,0), linx);
    		Imgproc.line(mGreyMat, new Point(0,hh-linx), new Point(ww/2-linx,hh), new Scalar(0,0,0), linx);
    	}
    	//Imgproc.watershed(mResultMat, mResultMat);
    	if (mSearchMode<=9) {
        	Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY+Imgproc.THRESH_OTSU);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY_INV);
        	//Imgproc.threshold(mGreyMat, mGreyMat, mLevel, 255, Imgproc.THRESH_BINARY_INV+Imgproc.THRESH_OTSU);
        	
    	}

		Imgproc.resize(mGreyMat, mGreyMat, new Size(w,h));
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
//        Imgproc.resize(mContour, mContour, new Size(w,h));
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
    	double x1 = 0;
    	double y1 = 0; 
    	if (mContour!=null) {
        	Moments mo = new Moments();
    		//mo = Imgproc.moments(mContour);
    		mo = contourMoments(mContour);
    		x = (mo.get_m10() / mo.get_m00());
    		y = (mo.get_m01() / mo.get_m00());
    		centPoint = new Point(x,y);
    		if (mOrientation==2) mDirection = (int)Math.round(Math.toDegrees(Math.atan((x-(w/2))/(h-lin-y))));
    		else mDirection = (int)Math.round(Math.toDegrees(Math.atan((h/2-y)/(w-2*lin-x))));
        	//Imgproc.drawContours(rgbaImage, mContours, 0, new Scalar(255,255,255), Core.FILLED);
        	//Imgproc.drawContours(mRgba, mContours, 0, new Scalar(255,0,0), 2);
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
        		//if (i>0) Core.line(mRgba, new Point(x0, y0), new Point(x1, y1), new Scalar(0,0,255), 1);
        	}
    		//Core.circle(mRgba, topPoint, 6, new Scalar(255,0,0), -1);
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
    	//Scalar mColor = new Scalar(0,255,0); // yellow
		//if (mDirection<-limit1 || mDirection>limit1) mColor = new Scalar(0,0,255); // blue
		//if (mDirection<-limit2 || mDirection>limit2) mColor = new Scalar(255,0,0); // red
		//Core.circle(mRgba, new Point(x, y), 5, new Scalar(255,49,0,255));
		if (mOrientation==2) {
			// landscape
			//Core.line(mRgba, new Point(x,y), new Point(w/2,h-lin), mColor, 3);
			//Core.line(mRgba, new Point(topPoint.x,topPoint.y), new Point(w/2,h-lin), new Scalar(255,255,0), 1);
			mLeftOK = 0;
			if (mGreyMat.get(2*h/3,w/4)[0]>0.5) mLeftOK = 1;
			mRightOK = 0;
			if (mGreyMat.get(2*h/3,3*w/4)[0]>0.5) mRightOK = 1;
		} else {
			// portrait
			//Core.line(mRgba, new Point(x,y), new Point(w-2*lin,h/2), mColor, 3);
			//Core.line(mRgba, new Point(topPoint.x,topPoint.y), new Point(w-2*lin,h/2), new Scalar(255,255,0), 1);
			mLeftOK = 0;
			if (mGreyMat.get(3*h/4,2*w/3)[0]>0.5) mLeftOK = 1;
			mRightOK = 0;
			if (mGreyMat.get(h/4,2*w/3)[0]>0.5) mRightOK = 1;
		}
		mCenterOK = 0;
		if (mGreyMat.get(h/2,w/2)[0]>0.5) mCenterOK = 1;
    	//mRgba.copyTo(mResultMat);
    }

    public Mat getResultMat() {
        return new Mat(h,w,0);
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }

    public int getDirection() {
        return mDirection;
    }

    public Point getCentPoint() {
        return centPoint;
    }

    public Point getTopPoint() {
        return topPoint;
    }

    public int getTopDirection() {
        return mTopDirection;
    }

    public int getTopHeight() {
        return mTopHeight;
    }

    public int getLeftOK() {
        return mLeftOK;
    }

    public int getRightOK() {
        return mRightOK;
    }

    public int getCenterOK() {
        return mCenterOK;
    }
}
