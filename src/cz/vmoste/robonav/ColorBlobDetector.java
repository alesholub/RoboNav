package cz.vmoste.robonav; 

import java.util.ArrayList;
import java.util.Iterator;
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
    private static double mMinContourArea = 0.7;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
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

//	private int p1 = 200;
//	private int p2 = 200;
//	private int p3 = 150;
//	private int p4 = 100;
//	private int p5 = 150;

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
        mMinContourArea = area;
    }

    public void setMinArea(double area) {
        minArea = area;
    }

    public void process(Mat mRgba) {
    	// detect one or two largest areas with defined color
    	// return contours of this areas (draw contours to original image, but not in searchMode 2 [RoboOrienteering])
    	// return direction to the largest area
    	// return central point of the largest area
    	// return bounding rectangle of the largest area
        //if (mSearchMode>=0) return;
        
    	Imgproc.pyrDown(mRgba, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
		MatOfPoint mContour0 = null;
		MatOfPoint mContour1 = null;
    	int w = mRgba.width();
    	int h = mRgba.height();
//    	p1 = w/3;
//    	p2 = p1*2;
//    	p3 = w/2;
//    	p4 = h/4;
//    	p5 = h/2;
    	//String mTxt1 = ""+mLowerBound.toString();
    	//String mTxt2 = ""+mUpperBound.toString();
       	//int textSize = Core.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
    	//Core.putText(mRgba, ""+mTxt1, new Point(w/4,h/3), 1, 2, new Scalar(255,255,50), 2);
    	//Core.putText(mRgba, ""+mTxt2, new Point(w/4,h/2), 1, 2, new Scalar(255,255,50), 2);

        mBoundingRectangle = new Rect(w/2,h/2,1,1);
        double maxArea = 0;
        //minArea = w/10;
    	int corner = w/8;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
                mContour1 = mContour0;
                mContour0 = wrapper;
        }
        // only two biggest contours must left
        contours.clear();
        if (mContour0!=null) contours.add(mContour0);
        if (mContour1!=null) contours.add(mContour1);

        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        if (maxArea>minArea) {
        	each = contours.iterator();
        	while (each.hasNext()) {
        		MatOfPoint contour = each.next();
        		if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
        			Core.multiply(contour, new Scalar(4,4), contour);
        			mContours.add(contour);
        		}
        	}
        }
        Scalar CONTOUR_COLOR = new Scalar(255,0,0,255);
		if (mSearchMode==20) {
    	  Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR, 2);
		}

//        Scalar mBlobColorRgba = converScalarHsv2Rgba(mHsvColor);
//    	Mat colorLabel = mRgba.submat(0, corner, 0, corner);
//    	colorLabel.setTo(mBlobColorRgba);

//    	Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
//    	if (mSearchMode!=2) mSpectrum.copyTo(spectrumLabel);
//    	Moments mo = new Moments();
    	int x = w/2;
    	int y = h/2;
//    	int x2 = 0;
//    	int y2 = 0;
		double x3 = x;
		double y3 = y;
		mTopPoint = new Point(x3, y3);
		direction = 0;
    	if (contours.size()>0) {
    		// set direction by position of first two contours
    		MatOfPoint contour = contours.get(0);
    		mBoundingRectangle = Imgproc.boundingRect(contour);
        	//mTxt1 = ""+contour.cols()+" / "+mBoundingRectangle.toString();
        	//Core.putText(mRgba, ""+mTxt1, new Point(w/4,h/3), 1, 2, new Scalar(255,255,50), 2);
        	//Core.rectangle(mRgba, mBoundingRectangle.tl(), mBoundingRectangle.br(),  new Scalar(0,255,0), 3);
//    		mo = Imgproc.moments(contour);
//    		x = (int) (mo.get_m10() / mo.get_m00());
//    		y = (int) (mo.get_m01() / mo.get_m00());
    		if (mBoundingRectangle.height<(h/200) || (mBoundingRectangle.x<=(2*corner) && mBoundingRectangle.y<=(2*corner))) {
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
//    		if (contours.size()>1 && mSearchMode<1) {
//    			// process second contour
//    			contour = contours.get(1);
//    			mo = Imgproc.moments(contour);
//    			x2 = (int) (mo.get_m10() / mo.get_m00());
//    			y2 = (int) (mo.get_m01() / mo.get_m00());
//    			Core.circle(mRgba, new Point(x2, y2), 4, new Scalar(255,49,0,255));
//    			// center of both contours (average)
//    			x3 = (x+x2)/2;
//    			y3 = (y+y2)/2;
//    		}
    		mTopPoint = new Point(x3, y3);
        	//mTxt1 = ""+mTopPoint.toString();
        	//Core.putText(mRgba, ""+mTxt1, new Point(w/4,h/3), 1, 2, new Scalar(255,255,50), 2);
//			Core.circle(mRgba, mTopPoint, 6, new Scalar(255,49,0,255),2);
//			Point endPoint = new Point(w/2,h);
			if (mOrientation==2) {
				// landscape
        		direction = (int)Math.round(Math.toDegrees(Math.atan((x3-(w/2))/(h-y3))));
			} else {
				// portrait
        		direction = (int)Math.round(Math.toDegrees(Math.atan((h/2-y3)/(w-x3))));
//        		endPoint = new Point(w,h/2);
			}
//    		if (mSearchMode==4) {
//    			// blob avoid
//        		if (contours.size()==1) {
//        			direction = -direction;
//        		}
//    		}
//	    	Scalar mColor = new Scalar(0,255,0); // green
//			if (direction<-limit1 || direction>limit1) mColor = new Scalar(0,0,255); // blue
//			if (direction<-limit2 || direction>limit2) mColor = new Scalar(255,0,0); // red
//	    	Core.line(mRgba, new Point(x3,y3), endPoint, mColor, 3);
//        	// 2 = RoboOrienteering (orange blob search, payload drop)
//        	// 4 = Blob Avoid (blob color is set by touch)
//        	// 5 = Blob Follow (color blob follow)
//    		if (mSearchMode==4) {
//    			// blob avoid
//    			if (y>p4 && x<p2 && x>p1) {
//    				//y2 = p4;
//    				y2 = p5;
//    				if (x<p3) p3 = p2;
//    			}
//    			if (Math.abs(direction)<=limit2) direction = 0;
//    			else direction = -direction;
//    		} else {
//    			// blob follow (and RoboOrienteering)
//    		} 
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

//    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
//        Mat pointMatRgba = new Mat();
//        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
//        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);
//
//        return new Scalar(pointMatRgba.get(0, 0));
//    }
    
}
