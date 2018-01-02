package cz.vmoste.robonav; 

import java.util.ArrayList;
import java.util.Iterator;
//import java.util.Iterator;
import java.util.List;
//import java.util.Vector;




import org.opencv.core.Core;
import org.opencv.core.CvType;
//import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
//import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.FeatureDetector;
//import org.opencv.features2d.KeyPoint;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class ObstacleDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    private Scalar mHsvColor = new Scalar(0);
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.7;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private List<Point> mObstacles = new ArrayList<Point>();
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

	private int p1 = 200;
	private int p2 = 200;
	private int p3 = 150;
	private int p4 = 100;
	private int p5 = 150;

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
        
        mHsvColor = hsvColor;

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
    	// detect KeyPoints
        Imgproc.pyrDown(mRgba, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);
//    	Imgproc.threshold(mPyrDownMat, mPyrDownMat, mLevel/2+20, 255, Imgproc.THRESH_BINARY);
//    	MatOfKeyPoint objectKeyPoints = new MatOfKeyPoint();
//        FeatureDetector featureDetector = FeatureDetector.create(FeatureDetector.SURF);
//        featureDetector.detect(mPyrDownMat, objectKeyPoints);
//        mObstacles = objectKeyPoints.toList();
//    	Imgproc.goodFeaturesToTrack(mPyrDownMat, features, 4, 1.0, 10.0);
//		mResultMat = mPyrDownMat;
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

    public List<Point> getObstacles() {
        return mObstacles;
    }

    public Rect getBoundingRectangle() {
        return mBoundingRectangle;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
    
}
