package cz.vmoste.robonav; 

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import org.opencv.android.BaseLoaderCallback;
//import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.SharedPreferences;
import android.location.Location;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.speech.tts.TextToSpeech;
import android.text.format.Time;
import android.util.FloatMath;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Toast;
import cz.vmoste.robonav.R;

public class NavigationActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "VisualNavigation::Activity";

	private static int tstText = 0;
	private static String tmpText = "";
	private static String toRgba = "none";
    private static final String SEARCHMODE_KEY = "searchmode";
    private static final String THRESHOLDLIMIT_KEY = "thresholdlimit";
	
	private TextToSpeech tts;
	
    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private Scalar               mBlobColorHsv1;
    private Scalar               mBlobColorHsv2;
    private ColorBlobDetector    mColorDetector;
    private RoadDetector         mRoadDetector;
    private LineDetector         mLineDetector;
    private byte                 mCommand = '-';
    private byte                 mPrevCommand = '-';
    private byte[]               out = new byte[1];
    private byte[]               out1 = new byte[1];
	private int w = 300;
	private int h = 200;
	private double siz = 0.8;
	private int pos = 100;
	private int wi = 1;
	private boolean enabled = true;
	private static int mLevel = 135;
    private int mod2 = mLevel % 2;
    //private int mod4 = mLevel % 4;
    private int mod6 = mLevel % 6;
	private static Scalar BLACK = new Scalar(0,0,0,255);
	private static Scalar RED = new Scalar(255,0,0,255);
	private static Scalar GREEN = new Scalar(0,255,0,255);
	private static Scalar BLUE = new Scalar(0,0,255,255);
	private static Scalar WHITE = new Scalar(255,255,255,255);
	private static Scalar YELLOW = new Scalar(255,255,0,255);
	private Point pt1, pt2, pt3, pt4, pt5, pt6, pt7, pt8;
	private int corner = 40;
	private int corner2 = 45;
	private Scalar mColor = BLACK;
	private Scalar mColor1 = BLUE;
	private Scalar mColor2 = BLUE;
	private static int limit1 = 5;
	private static int limit2 = 15;
	private String tx = "";
	private Size textSize;
	private int[] baseline = null;
	private static double minArea = 0.5;
	private static String[] sm = new String[]{"MC","RR","RO","RT","BA","BF","LD","KH","BR","RC"};
	private static int mOrientation = 0;
	private static String mHSV1 = "";
	private static String mHSV2 = "";
    private static String address = "00:00:00:00:00:00";
    private static String address2 = "00:00:00:00:00:00";
	private static String commandsTable0 = "lrswfbhkpt";
	private static String commandsTable = "lrswfbhkpt";
	private static Map<Character, Character> commandsMap = new HashMap<Character, Character>();
	private SharedPreferences mPrefs;
	private int direction = 0;
	private int topDirection = 0;
    private int directionTrend = 0;
	private int directionNum = 0;
	private static int azimuth = 0;
	//private static int initialAzimuth = 0;
	private static int azimuthLimit = 25;
	private static int azimuthDifference = 0;
	private static Time now = new Time();
	private static long azimuthValidTime = 0;
	private static int azimuthValid = 0;
	private static int btValid = 0;
	private static int btPwm = 0; // PWM from bluetooth
	private static int btRng = 99; // center sonar range from bluetooth
	private static int btDst = 0; // distance from bluetooth
	private static int btSpd = 0; // speed from bluetooth
    private static double btLat = 0.0; // waypoint latitude from BT connection
    private static double btLon = 0.0; // waypoint longitude from BT connection
	private static int btRngLeft = 99; // left sonar range from bluetooth
	private static int btRngRight = 99; // right sonar range from bluetooth
	private static int btRngBack = 99; // IR range from bluetooth
    private boolean  stopped = false;
    private static double lat = 0.0;
    private static double lon = 0.0;
    private static float accuracy = 888;
    private static float bearing = 888;
	private static String stav = "???";
    private static double latOK = 0.0; // best estimation of lattitude
    private static double lonOK = 0.0; // best estimation of longitude
    private static double azimuthOK = 0.0; // best estimation of azimuth (course)
    private static int distanceOK = 0; // best estimation of distance (traveled)
	private static List<Point> path = new ArrayList<Point>(); // desired path
	private static int wp = 0; // waypoint number
    private static double wpLat = 0.0; // waypoint latitude
    private static double wpLon = 0.0; // waypoint longitude
    private static double wpDist = 0.0; // waypoint distance
    private static double wpAzim0 = 0.0; // azimuth from previous waypoint to next waypoint
    private static double wpAzim1 = 0.0; // azimuth from actual GPS position to next waypoint
	private static int wpMode = 0; // waypoint mode (or points)
    private static int lastDist = 0; // distance at last waypoint
	private static List<Point> goals = new ArrayList<Point>(); // waypoints
	private static List<Point> points = new ArrayList<Point>(); // map points
	private static List<int[]> edges = new ArrayList<int[]>(); // map edges (point1, point2, dist, azim)
	private static List<int[]> wpModes = new ArrayList<int[]>(); // waypoint modes (mode)
	private static double minLat = 999.0;
	private static double minLon = 999.0;
	private static double maxLat = -999.0;
	private static double maxLon = -999.0;
	private static double centLat = 0.0; // center lattitude (degrees)
	private static double centLon = 0.0; // center longitude (degrees)
	private static double multLat = 1.0; // map lattitude multiplier (degrees to pixels)
	private static double multLon = 1.0; // map longitude multiplier (degrees to pixels)
	private static int inputMode = 0; // input mode (0=colors, 1=new_path)
	private static String startTime = "";
	private static long startTimeMilis = 0;
	private static int runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
	private static String pom = "";
	private static long tmpSec = 0;
	private static long tmpSec0 = 0;
	private static int averageDirection = 0;
	private static int goalReached = 0;
	private static int avoiding = 0;
	private static String txtCommand = "";
	private static int averageTopPoint = 0;
    private int topPointTrend = 0;
	private int lastX = -999;
	private int lastY = -1;
	private static String state = "normal";
    private int topHeight = 0;
    private int cameraDirection = 0;
    private int cameraDiff = 0;
    private int cameraProbability = 100;
    private int actualDist = 0;
    private double wpDist1 = 0;
    private double azimDiff = 0;
    private double azimDiff1 = 0;
    private int azimDiffOK = 0;
    private double wpDistOK = 0;
    private int move = 0;
	private Time lastDown = new Time();
	private long touchTime = 0;
	private float oldDist = 1f;
	private float newDist = 1f;
	private int touchMode = 0;
	private int counter = 0;
	private int debugMode = 0;
	private int buttonTouched = 0;
	private int x = 1;
	private int y = 1;
	private int ex = 1;
	private int ey = 1;
	private int vw = 1;
	private int vh = 1;
	private String mTxt1 = "";
	private String mTxt2 = "";
	private String mTxt3 = "";
	private String mTxt4 = "";
	private String mTxt5 = "";
	private String mTxt6 = "";
	private int mTargetAzimuth = 999;
	private int compassAzimuth = 0;
	private int numCleanCommands = 0;
	private static double drivingSignal = 0.0;
	private int lastDirectionOK = 0;
	private int directionOK = 0;
	private int mLeftOK = 1;
	private int mRightOK = 1;
	
	private static int searchMode = 0;
	private static int mArea = 50;
	private static int voiceOutput = 1;
	private static int azimLimit = 20;

    private Compass compass;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(NavigationActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public NavigationActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    void setLevel() {
    	mRoadDetector.setLevel(mLevel);
    	mLineDetector.setLevel(mLevel);
    	mColorDetector.setLevel(mLevel);
    }
    
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        //if (tts!=null) Toast.makeText(getApplicationContext(), "TTS OK", Toast.LENGTH_SHORT).show();
        //else Toast.makeText(getApplicationContext(), "TTS ERR", Toast.LENGTH_SHORT).show();
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        compass = new Compass(this);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        //mOpenCvCameraView.setMaxFrameSize(640, 480);
        //mOpenCvCameraView.setMinimumWidth(600);
        //mOpenCvCameraView.setMinimumHeight(300);
        //LayoutParams mLayoutParams = (LayoutParams) mOpenCvCameraView.getLayoutParams();
        //mLayoutParams.width = 640;
        mOpenCvCameraView.setCvCameraViewListener(this);
    	mOpenCvCameraView.enableFpsMeter();
    	mOpenCvCameraView.offsetTopAndBottom(0);
    	init();
    	stopped = false;
        appendLog("date_time;mCommand;searchMode;direction;topDirection;cameraDirection;azimuth;azimuthValid;btPwm;btRng;btRngLeft;btRngRight;btRngBack;btDst;btSpd;limit1;limit2;mLevel;azimuthLimit;mArea;minArea;btLat;btLon;stav;lat;lon;bearing;accuracy;wp;wpLat;wpLon;wpMode;wpDist;wpAzim0;wpDist1;wpAzim1;azimuth;azimDiff;azimDiff1;actualDist;lastDist;");
        sendCommand(); // start timer
    }

    @Override
    public void onPause()
    {
        if(tts !=null){
            tts.stop();
            tts.shutdown();
         }
        appendLog("pause");
        super.onPause();
        compass.stop();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
        //Toast.makeText(getApplicationContext(), "onResume", Toast.LENGTH_SHORT).show();
        tts = new TextToSpeech(getApplicationContext(), 
      	      new TextToSpeech.OnInitListener() {
      	      @Override
      	      public void onInit(int status) {
      	         if(status != TextToSpeech.ERROR){
      	             tts.setLanguage(Locale.US);
         	             Toast.makeText(getApplicationContext(), "TTS OK "+status, Toast.LENGTH_SHORT).show();
      	            } else {
            	             Toast.makeText(getApplicationContext(), "TTS ERR "+status, Toast.LENGTH_SHORT).show();
      	            }
      	         }
      	      });
        //if (tts!=null) Toast.makeText(getApplicationContext(), "TTS OK", Toast.LENGTH_SHORT).show();
        //else Toast.makeText(getApplicationContext(), "TTS ERR", Toast.LENGTH_SHORT).show();
        compass.start();
    	stopped = false;
        appendLog("resume");
    }

    @Override
    public void onStop() {
        super.onStop();
        appendLog("stop");
        compass.stop();
    	stopped = true;
        finish();
    }


    public void onDestroy() {
        mRgba.release();
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        appendLog("destroy");
    	stopped = true;
        finish();
    }

    public void onCameraViewStarted(int width, int height) {
        //Toast.makeText(getApplicationContext(), "onCameraViewStarted", Toast.LENGTH_SHORT).show();
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mColorDetector = new ColorBlobDetector();
        mRoadDetector = new RoadDetector();
        mLineDetector = new LineDetector();
        // autotouch (green)
        //Toast.makeText(getApplicationContext(), "autotouch", Toast.LENGTH_SHORT).show();
        //mColorDetector.setColorRadius(new Scalar(25,50,50,0));
        mBlobColorHsv = new Scalar(75,150,150,0); // soft green
        mBlobColorHsv1 = new Scalar(100,255,220,0); // strong green
        mBlobColorHsv2 = new Scalar(250,200,200,0); // orange
        mHSV1 = mBlobColorHsv1.toString();
        mHSV2 = mBlobColorHsv2.toString();
        readPrefs();
        compass.start();
        mBlobColorHsv = mBlobColorHsv1;
        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv1);
        mColorDetector.setHsvColor(mBlobColorHsv1);
        if (searchMode==2) {
        	// RoboOrienteering (initial orange blob)
            mBlobColorHsv = mBlobColorHsv2;
            mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv2);
            mColorDetector.setHsvColor(mBlobColorHsv2);
        }
        mRoadDetector.setHsvColor(mBlobColorHsv);
        mLineDetector.setHsvColor(mBlobColorHsv);
    	w = mRgba.width();
    	h = mRgba.height();
    	corner = w/8;
    	corner2 = 2;
    	pt1 = new Point(w-corner,corner);
    	pt2 = new Point(w,0);
    	pt3 = new Point(w-2*corner,h-corner);
    	pt4 = new Point(w,h);
    	pt5 = new Point(w-corner,h/2-corner-corner2);
    	pt6 = new Point(w,h/2-corner2);
    	pt7 = new Point(w-corner,h/2+corner+corner2);
    	pt8 = new Point(w,h/2+corner2);
    	pos = h - 10;
    	siz = 0.8;
    	if (w>400) {
    		siz = 1.4;
    		wi = 2;
    	}
    	mRoadDetector.setLevel(mLevel);
    	mRoadDetector.setLimits(limit1,limit2);
    	mRoadDetector.setOrientation(mOrientation);
    	mColorDetector.setLevel(mLevel);
    	mColorDetector.setLimits(limit1,limit2);
    	mColorDetector.setOrientation(mOrientation);
        mColorDetector.setMinContourArea(minArea);
        mColorDetector.setMinArea(mArea);
    	mLineDetector.setLevel(mLevel);
    	mLineDetector.setLimits(limit1,limit2);
    	mLineDetector.setOrientation(mOrientation);
    	readMap("RoboNavMap.txt");
    	goals = readPoints("RoboNavGoals.txt");
    	path = readPoints("RoboNavPath.txt");
    	computeNextWaypoint(1);
    	now.setToNow();
    	startTimeMilis = now.toMillis(false)+300000;
    	startTimeMilis = (startTimeMilis/300000)*300000;
    	setStartTime();
    	runMode = 0; // wait for start time
    }

    public void onCameraViewStopped() {
        appendLog("cameraStop");
        finish();
    }

    public boolean onTouchEvent(MotionEvent event) {
    	//tts.speak("event", TextToSpeech.QUEUE_ADD, null);
    	
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        vw = mOpenCvCameraView.getWidth();
        vh = mOpenCvCameraView.getHeight();

        ex = (int)event.getX();
        ey = (int)event.getY();
        x = ex*cols/vw;
        y = ey*rows/vh;

        switch (event.getAction() & MotionEvent.ACTION_MASK) {
		case MotionEvent.ACTION_DOWN:
        	//tts.speak("down", TextToSpeech.QUEUE_ADD, null);
			move = 0;
			touchMode = 1; // drag
			break;
		case MotionEvent.ACTION_UP:
        	//tts.speak("up", TextToSpeech.QUEUE_ADD, null);
			touchTime = event.getEventTime() - event.getDownTime();
    		//Toast.makeText(getApplicationContext(), ""+touchTime+" / "+event.getPressure(), Toast.LENGTH_SHORT).show();
    		//Toast.makeText(getApplicationContext(), ""+inputMode+" / "+move+" / "+touchMode+" / "+buttonTouched, Toast.LENGTH_SHORT).show();
            if (inputMode>0 && move==0 && buttonTouched==0) {
            	// new waypoint defined
                double xLat = 0; 
                double xLon = 0; 
            	double tmpLon = centLon + ((x-w/2))/multLon;
            	double tmpLat = centLat - ((y-h/2))/multLat;
            	double diff = 0.0;
            	double minDiff = 999.0;
            	int minIndex = 0;
                for (int i=0; i<points.size(); i++) {
                	xLon = points.get(i).x;
                	xLat = points.get(i).y;
                	diff = Math.abs(tmpLon-xLon)+Math.abs(tmpLat-xLat);
                	if (diff<minDiff) {
                		minDiff = diff;
                		minIndex = i;
                	}
                }
                if (minDiff<0.0003) {
                	tmpLon = points.get(minIndex).x;
                	tmpLat = points.get(minIndex).y;
                }
            	path.add(new Point(tmpLon,tmpLat));
            	int xMode = 0;
            	//if (event.getPressure()>0.35) xMode = 1;
            	if (touchTime>300) xMode = 1;
            	wpModes.add(new int[]{xMode});
                lastX = -999;
            	return false;
            }
            lastX = -999;
        	buttonTouched = 0;
        	move = 0;
			break;
		case MotionEvent.ACTION_POINTER_DOWN:
        	//tts.speak("pointer down", TextToSpeech.QUEUE_ADD, null);
			if (buttonTouched==0) {
				move = 0;
				oldDist = spacing(event);
				if (oldDist > 10f) {
					touchMode = 2; // zoom
				}
			}
			break;
		case MotionEvent.ACTION_POINTER_UP:
        	//tts.speak("pointer up", TextToSpeech.QUEUE_ADD, null);
            lastX = -999;
			touchMode = 0; // none
        	buttonTouched = 0;
			//move = 0;
			break;
		case MotionEvent.ACTION_MOVE:
        	//tts.speak("move", TextToSpeech.QUEUE_ADD, null);
			if (buttonTouched==0) {
				if (touchMode==2) {
					// zoom
					newDist = spacing(event);
					if (newDist > 10f) {
			            float scale = newDist / oldDist;
			            oldDist = newDist;
			            multLon = multLon * scale;
			            multLat = multLat * scale;
			        }
					move = 2;
				} else {
					// move
			    	int dx = 0;
		            int dy = 0;
		            if (lastX>-999) {
		            	dx = x - lastX;
		            	dy = y - lastY;
		            	centLon = centLon - dx/multLon;
		            	centLat = centLat + dy/multLat;
		            }
	            	if ((dx+dy)>2 && move==0) move = 1;
		            lastX = x;
		            lastY = y;
				}
			}
			break;
		}
		return true;
    }

    public boolean onTouch(View v, MotionEvent event) {
    	//tts.speak("touch", TextToSpeech.QUEUE_ADD, null);
		lastDown.setToNow();
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        vw = mOpenCvCameraView.getWidth();
        vh = mOpenCvCameraView.getHeight();

        ex = (int)event.getX();
        ey = (int)event.getY();
        x = ex*cols/vw;
        y = ey*rows/vh;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");
        
        if ((x>(cols-corner)) && (y<corner)) {
        	// mute corner (top right)
    	    if (voiceOutput>0) voiceOutput = 0; else voiceOutput = 1;
        	if (voiceOutput>0) tts.speak("voice", TextToSpeech.QUEUE_ADD, null);
        	buttonTouched = 1;
    	    return false;
        }
        else if ((x<corner) && (y>(rows-corner))) {
        	// debugMode corner (bottom left)
    	    if (debugMode>0) {
    	    	debugMode = 0;
    	    }
    	    else {
    	    	debugMode = 1;
    	    }
        	if (voiceOutput>0) tts.speak("debug "+debugMode, TextToSpeech.QUEUE_ADD, null);
        	buttonTouched = 2;
    	    return false;
        }
        else if ((x<corner) && (y<corner) && searchMode>0) {
        	// inputMode corner (top left)
    	    if (inputMode>0) {
    	    	inputMode = 0;
    	    	path = readPoints("RoboNavPath.txt");
    	    	wp = 0;
    	    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	    	goalReached = 0; // we should be at start now
    	    	state = "normal";
    	    	computeNextWaypoint(1);
    	    }
    	    else {
    	    	inputMode = 1;
    	    	path.clear();
    	    	wpModes.clear();
    	    }
        	if (voiceOutput>0) tts.speak("input mode "+inputMode, TextToSpeech.QUEUE_ADD, null);
        	buttonTouched = 3;
    	    return false;
        }
        else if ((x>(cols-2*corner)) && (y>(rows-corner)) && inputMode<1) {
        	// searchmode corner (bottom right)
        	searchMode++;
    	    if (searchMode>6) searchMode = 0;
    	    if (searchMode==2) {
                mBlobColorHsv = mBlobColorHsv2;
    	    	mColorDetector.setHsvColor(mBlobColorHsv2);
    	    	mHSV2 = mBlobColorHsv2.toString();
    	    }
    	    else {
                mBlobColorHsv = mBlobColorHsv1;
    	    	mColorDetector.setHsvColor(mBlobColorHsv1);
    	    	mHSV1 = mBlobColorHsv1.toString();
    	    }
            mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);
        	if (voiceOutput>0) tts.speak("search mode "+searchMode, TextToSpeech.QUEUE_ADD, null);
        	buttonTouched = 4;
    	    return false;
        }
        else if ((x<corner) && (y>(rows/2-corner/2)) && (y<(rows/2+corner/2))) {
        	// save area (left center)
        	if (voiceOutput>0) tts.speak("saving", TextToSpeech.QUEUE_ADD, null);
        	saveConfig();
        	// save path
        	String pathText = "";
            for (int i=0; i<path.size(); i++) {
            	pathText += ""+i+" "+path.get(i).y+" "+path.get(i).x+" "+wpModes.get(i)[0]+"\n";
            }
        	writeToFile("RoboNavPath.txt",pathText);
        	buttonTouched = 5;
    	    return false;
        }
        else if ((x>=(cols-corner)) && (y<(rows/2)) && (y>=(rows/2-corner-corner2)) && searchMode>0) {
        	// plus rectangle (above center right)
        	buttonTouched = 6;
        	if (voiceOutput>0) tts.speak("plus", TextToSpeech.QUEUE_ADD, null);
        	if (inputMode>0) {
        		// increase startTime
        		startTimeMilis += 60000;
        		setStartTime();
        		return false;
        	}
        	if (searchMode==6) {
                mLevel = mLevel + 10;
                if (mLevel>255) mLevel = 255;
                mRoadDetector.setLevel(mLevel);
                mLineDetector.setLevel(mLevel);
        	}
        	else if (searchMode==1 || searchMode==3) {
                mLevel = mLevel + 1;
                if (mLevel>255) mLevel = 255;
                mRoadDetector.setLevel(mLevel);
                mLineDetector.setLevel(mLevel);
        	}
        	else {
                //minArea = (double)Math.round((minArea + 0.1)*100)/100;
                //if (minArea>1) minArea = 1;
                //mColorDetector.setMinContourArea(minArea);
                limit1 = limit1 + 1;
                if (limit1>25) limit1 = 25;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mLineDetector.setLimits(limit1,limit2);
        	}
            if (mColor1==BLUE) mColor1 = YELLOW; else mColor1 = BLUE;
        	mColor1 = RED;
    	    return false;
        }
        else if ((x>=(cols-corner)) && (y>=(rows/2)) && (y<=(rows/2+corner+corner2)) && searchMode>0) {
        	// minus rectangle (below center right)
        	buttonTouched = 7;
        	if (voiceOutput>0) tts.speak("minus", TextToSpeech.QUEUE_ADD, null);
        	if (inputMode>0) {
        		// decrease startTime
        		startTimeMilis -= 60000;
        		setStartTime();
        		return false;
        	}
        	if (searchMode==6) {
                mLevel = mLevel - 10;
                if (mLevel<0) mLevel = 0;
                mRoadDetector.setLevel(mLevel);
                mLineDetector.setLevel(mLevel);
        	}
        	else if (searchMode==1 || searchMode==3) {
                mLevel = mLevel - 1;
                if (mLevel<0) mLevel = 0;
                mRoadDetector.setLevel(mLevel);
                mLineDetector.setLevel(mLevel);
        	}
        	else {
                //minArea = (double)Math.round((minArea - 0.1)*100)/100;
                //if (minArea<0) minArea = 0;
                //mColorDetector.setMinContourArea(minArea);
                limit1 = limit1 - 1;
                if (limit1<0) limit1 = 0;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mLineDetector.setLimits(limit1,limit2);
        	}
            if (mColor2==BLUE) mColor2 = YELLOW; else mColor2 = BLUE;
        	mColor2 = RED;
    	    return false;
        }
        else if (searchMode==0) {
        	// manual control
        	buttonTouched = 0;
            if (x<=(1.5*w/10)) {
            	// turn
    	    	out[0] = 't';
            	buttonTouched = 11;
            } else if (x<=(3*w/10)) {
            	// speed
    	    	out[0] = (byte)('0'+10*(h-y-h/20)/h);
            	buttonTouched = 100+10*(h-y)/h;
            } else if (x<=(5*w/10)) {
            	// forward / h / k
            	buttonTouched = 21;
    	    	out[0] = 'f';
    	    	if (y>(2*h/3)) {
                	buttonTouched = 22;
    	    		out[0] = 'h'; // xleft
    	    	}
    	    	else if (y<(1*h/3)) {
                	buttonTouched = 23;
    	    		out[0] = 'k'; // xright
    	    	}
            } else if (x>=(7*w/10)) {
            	// backward
            	buttonTouched = 31;
    	    	out[0] = 'b';
    	    	if (y<(1*h/3)) {
                	buttonTouched = 35;
    	    		out[0] = 'p'; // payload drop
    	    	}
            } else if (y>(2*h/3)) {
            	// left
            	buttonTouched = 32;
    	    	out[0] = 'l';
            } else if (y<(h/3)) {
            	// right
            	buttonTouched = 33;
    	    	out[0] = 'r';
            } else {
            	// stop
            	buttonTouched = 34;
    	    	out[0] = 's';
            }
          	if (!stopped && buttonTouched>0) {
             	writeCommand();
            	tellCommand((char)out[0]);
            }
    	    return false;
        }

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;
        
        if (searchMode==1) {
        	// targetAzimuth for RR
        	mTargetAzimuth = (int)azimuthOK;
        	return false;
        }
        
    	Rect touchedRect = new Rect();

        touchedRect.x = (x>4) ? x-4 : 0;
        touchedRect.y = (y>4) ? y-4 : 0;

        touchedRect.width = (x+4 < cols) ? x + 4 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+4 < rows) ? y + 4 - touchedRect.y : rows - touchedRect.y;

        Mat touchedRegionRgba = mRgba.submat(touchedRect);

        Mat touchedRegionHsv = new Mat();
        Imgproc.cvtColor(touchedRegionRgba, touchedRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        mBlobColorHsv = Core.sumElems(touchedRegionHsv);
        int pointCount = touchedRect.width*touchedRect.height;
        for (int i = 0; i < mBlobColorHsv.val.length; i++)
            mBlobColorHsv.val[i] /= pointCount;

        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);

        Log.i(TAG, "Touched rgba color: (" + mBlobColorRgba.val[0] + ", " + mBlobColorRgba.val[1] +
                ", " + mBlobColorRgba.val[2] + ", " + mBlobColorRgba.val[3] + ")");

        if (searchMode==4 || searchMode==5) {
        	mColorDetector.setHsvColor(mBlobColorHsv);
        	//mColorDetector.setMinArea(mArea);
        	mHSV1 = mBlobColorHsv.toString();
        } else if (searchMode==2) {
        	mBlobColorHsv2 = mBlobColorHsv;
        	mColorDetector.setHsvColor(mBlobColorHsv2);
        	//mColorDetector.setMinArea(mArea);
        	mHSV2 = mBlobColorHsv2.toString();
        }

        //Imgproc.resize(mColorDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        touchedRegionRgba.release();
        touchedRegionHsv.release();
        
        // Tell the camera how to display its preview images. Note that we actually don't want it to display
        // previews, so we have to turn off previews in an OS-specific way
        try {
        	if (enabled) {
        		enabled = false;
        		//mOpenCvCameraView.setEnabled(enabled);
        		//mOpenCvCameraView.disableView();
        		//mOpenCvCameraView.setKeepScreenOn(enabled);
        		//mOpenCvCameraView.setVisibility(0);
            	WindowManager.LayoutParams params = this.getWindow().getAttributes();
            	params.screenBrightness = 0.004f; // OK
            	params.screenBrightness = -1;
            	this.getWindow().setAttributes(params);
        	} else {
        		enabled = true;
        		//mOpenCvCameraView.setEnabled(enabled);
        		//mOpenCvCameraView.enableView();
        		//mOpenCvCameraView.setKeepScreenOn(enabled);
        		//mOpenCvCameraView.setVisibility(1);
            	WindowManager.LayoutParams params = this.getWindow().getAttributes();
            	params.screenBrightness = -1;
            	this.getWindow().setAttributes(params);
        	}
            // for 3.0 and later, we create a SurfaceTexture and let the camera dump its picture there.
//            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.HONEYCOMB) {
//                myTexture = new SurfaceTexture(10);
//                mCamera.setPreviewTexture(myTexture);
//            }
//            // for older Android versions, we can just give a null preview display window
//            else {
//                mCamera.setPreviewDisplay(null);
//            }
        }
        catch (Exception e) {
            Log.e(TAG, "mCamera.setPreviewDisplay/setPreviewTexture fails: " + e);
        }

        return false; // don't need subsequent touch events
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();
      	if (stopped) return mRgba;
    	mRoadDetector.setSearchMode(searchMode);
    	mLineDetector.setSearchMode(searchMode);
    	mColorDetector.setSearchMode(searchMode);

    	// searchmode:
    	// 0 = Manual Control (direction, speed)
    	// 1 = RobotemRovne (road search, straight road, azimuth, odometry, crossing roads)
    	// 2 = RoboOrienteering (orange blob search, waypoints, payload drop)
    	// 3 = RoboTour (road search, waypoints, payload drop)
    	// 4 = Blob Avoid (blob color is set by touch, initial is green, waypoints, payload drop)
    	// 5 = Blob Follow (color blob follow, initial is orange, waypoints, payload drop)
    	// 6 = Line Detector (lines detection, waypoints, payload drop)
    	// 7 = Ketchup House (not implemented yet)
    	// 8 = Bear Rescue (not implemented yet)
    	// 9 = Robo Carts (not implemented yet)
    	
    	if (now.toMillis(false)<=(startTimeMilis+3000)) runMode = 0;
    	else runMode = 1;
    	if (searchMode<=1) runMode = 1;
        mod2 = mLevel % 2;
        //mod4 = mLevel % 4;
        mod6 = mLevel % 6;

    	if (inputMode>=0 && !stopped) {
            if (searchMode==0) {
            	// Manual Control
            	direction = 0;
                mRgba.setTo(new Scalar(0,0,0));
                //Core.transpose(mRgba, mRgba);
                //int mTmp = h;
                //h = w;
                //w = mTmp;
                int bw = w/5-2;
                int bh = h/3-2;
                int cw = 6*w/10;
                int ch = 5*h/6;
        		Point pta = new Point(cw-bw/2,ch-bh/2);
        		Point ptb = new Point(cw+bw/2,ch+bh/2);
        		mColor = new Scalar(64,64,64);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "l";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(64,64,64);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "r";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(64,64,64);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "s";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 2*w/10;
            	ch = 1*h/6;
        		pta.x = cw-bw/4; pta.y = ch-bh/2;
            	ch = 5*h/6;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "0";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/2;
            	tx = "5";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = 1*h/6;
            	tx = "9";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 4*w/10;
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "f";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = 5*h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "h";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "k";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 8*w/10;
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "b";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "p";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = (int)(0.5*w)/10;
            	ch = 1*h/6;
        		pta.x = cw-bw/4; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Core.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "t";
            	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Core.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
                //Core.transpose(mRgba, mRgba);
                //w = h;
                //h = mTmp;
            	//if (debugMode>0) Core.putText(mRgba, "o: "+xOffset+" / "+yOffset+" / "+w+" / "+h+" / "+x+" / "+y+" / "+ex+" / "+ey+" / "+vw+" / "+vh, new Point(4,(pos-100)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else if (searchMode==1) {
            	// Road detection (for RobotemRovne) ... direction from moments or topPoint
                mRoadDetector.process(mRgba);
                Mat mTemp = mRoadDetector.getResultMat();
                mRgba = mTemp;
                //mTemp.release();
            	direction = mRoadDetector.getDirection();
            	topDirection = mRoadDetector.getTopDirection();
            	topHeight = mRoadDetector.getTopHeight();
            	mLeftOK = mRoadDetector.getLeftOK();
            	mRightOK = mRoadDetector.getRightOK();
            	//Core.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Core.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Core.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else if (searchMode==3) {
            	// Road detection (for RoboTour) ... direction from topPoint or moments (merging)
                mRoadDetector.process(mRgba);
                mRgba = mRoadDetector.getResultMat();
                //int channel = (int)Math.floor(mLevel - (int)Math.floor(mLevel/8)*8);
            	direction = mRoadDetector.getDirection();
            	topDirection = mRoadDetector.getTopDirection();
            	topHeight = mRoadDetector.getTopHeight();
            	//Core.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+channel, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Core.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Core.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
    			// merging
            	//direction = topDirection + direction/2; // merging disabled
            } else if (searchMode==6) {
            	// Line detection (maybe universal) ... direction from topPoint only
                mLineDetector.process(mRgba);
                Mat mTemp = mLineDetector.getResultMat();
                mRgba = mTemp;
                //mTemp.release();
            	direction = mLineDetector.getDirection();
            	topDirection = direction;
            	topHeight = mLineDetector.getTopHeight();
            	//Core.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Core.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Core.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else {
            	// Color Blob Detection
            	// 2 = RoboOrienteering (orange blob search, payload drop)
            	// 4 = Blob Avoid (blob color is set by touch)
            	// 5 = Blob Follow (color blob follow)
            	mColorDetector.process(mRgba);
                mRgba = mColorDetector.getResultMat();
            	direction = mColorDetector.getDirection();
            	//topDirection = mColorDetector.getTopDirection();
            	//topHeight = mColorDetector.getTopHeight();
            	//Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+minArea+"/"+limit1+"/"+limit2+"/"+direction+"/"+x+"/"+y+color, new Point(4,(7*h/8)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Core.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+minArea+"/"+limit1+"/"+limit2+"/"+direction, new Point(4,(7*h/8)), 1, siz, new Scalar(255,255,150), wi);
            	//Core.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Core.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            }
        }
    	if (inputMode>0) {
        	// inputMode>0 => path defining (black background)
            mRgba.setTo(new Scalar(0,0,0));
        }
        if (searchMode!=0 && !stopped) {
        	// add main information to mRgba
        	//if (debugMode>=0) Core.putText(mRgba, ""+mLevel+" "+cameraDirection+" "+" "+Math.round(drivingSignal)+" "+mod4, new Point(w/6,h-h/6), 1, siz*2, new Scalar(255,255,50), wi*3);
        	mTxt1 = ""+String.format("%4d", (int)azimuthOK);
        	mTxt2 = ""+String.format("%4d", (int)mTargetAzimuth);
        	mTxt3 = ""+String.format("%4d", (int)azimDiffOK);
        	mTxt4 = ""+String.format("%4d", (int)drivingSignal);
        	mTxt5 = " "+(char)(out1[0] & 0xFF)+"/"+mLeftOK+"/"+mRightOK;
        	//mTxt5 = ""+String.format("%4d", (int)commandsMap.size());
        	//mTxt5 = ""+String.format("%4d", (int)azimLimit);
        	//mTxt5 = ""+String.format("%4d", (int)wpAzim0);
        	//mTxt6 = ""+String.format("%4d", (int)wpAzim1);
        	mTxt6 = ""+String.format("%4d", (int)mod6);
        	//mTxt5 = "a:"+String.format("%4d", (int)azimuthOK);
        	if (debugMode>=0) Core.putText(mRgba, ""+mTxt1+" "+mTxt2+" "+" "+mTxt3+" "+mTxt4+" "+mTxt5+" "+mTxt6, new Point(1,0.7*h), 1, siz*1.5, new Scalar(255,255,50), wi*2);
        	if (debugMode>=0) Core.putText(mRgba, ""+txtCommand, new Point(w/2,h-h/60), 1, siz*2, new Scalar(255,0,255), wi*3);
        	int ok = 0;
        	double tmpx = 0;
        	double tmpy = 0;
        	Scalar tmpColor = new Scalar(200,200,200);
        	// show command graphically
        	if (mOrientation==2) {
    			// landscape
        		tmpy = h*(0.75-0.6*Math.cos(drivingSignal/36));
        		tmpx = 0.5*w+0.6*h*Math.sin(drivingSignal/36);
    			Core.line(mRgba, new Point(0.5*w,0.75*h), new Point(tmpx,tmpy), tmpColor, 5);
    			Core.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,0,0), -1);
            	if (txtCommand=="xleft") {ok = 1; tmpx = w/4; tmpy = h/2; tmpColor = new Scalar(255,0,0);}
            	else if (txtCommand=="xright") {ok = 1; tmpx = 3*w/4; tmpy = h/2; tmpColor = new Scalar(255,0,0);}
            	else if (txtCommand=="forward") {ok = 1; tmpx = w/2; tmpy = h/4; tmpColor = new Scalar(0,255,0);}
            	else if (txtCommand=="straigh") {ok = 1; tmpx = w/2; tmpy = h/4; tmpColor = new Scalar(255,255,255);}
            	else if (txtCommand=="left") {ok = 1; tmpx = w/3; tmpy = h/3; tmpColor = new Scalar(0,0,255);}
            	else if (txtCommand=="right") {ok = 1; tmpx = 2*w/3; tmpy = h/3; tmpColor = new Scalar(0,0,255);}
            	else if (txtCommand=="stop") {ok = 1; tmpx = w/2; tmpy = h/2; tmpColor = new Scalar(0,0,0);}
            	else if (txtCommand=="back") {ok = 1; tmpx = w/2; tmpy = 0.9*h; tmpColor = new Scalar(255,0,0);}
    		} else {
    			// portrait
        		tmpx = h*(0.75-0.6*Math.cos(drivingSignal/36));
        		tmpy = 0.5*w+0.6*h*Math.sin(drivingSignal/36);
    			Core.line(mRgba, new Point(0.75*w,0.5*h), new Point(tmpx,tmpy), tmpColor, 5);
    			Core.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,0,0), -1);
    			//Core.line(mRgba, new Point(topPoint.x,topPoint.y), new Point(w-2*lin,h/2), new Scalar(0), 1);
            	if (txtCommand=="xleft") {ok = 1; tmpx = w/2; tmpy = 3*h/4; tmpColor = new Scalar(255,0,0);}
            	else if (txtCommand=="xright") {ok = 1; tmpx = w/2; tmpy = h/4; tmpColor = new Scalar(255,0,0);}
            	else if (txtCommand=="forward") {ok = 1; tmpx = w/4; tmpy = h/2; tmpColor = new Scalar(0,255,0);}
            	else if (txtCommand=="straigh") {ok = 1; tmpx = w/4; tmpy = h/2; tmpColor = new Scalar(255,255,255);}
            	else if (txtCommand=="left") {ok = 1; tmpx = w/3; tmpy = 2*h/3; tmpColor = new Scalar(0,0,255);}
            	else if (txtCommand=="right") {ok = 1; tmpx = w/3; tmpy = h/3; tmpColor = new Scalar(0,0,255);}
            	else if (txtCommand=="stop") {ok = 1; tmpx = w/2; tmpy = h/2; tmpColor = new Scalar(0,0,0);}
            	else if (txtCommand=="back") {ok = 1; tmpx = 0.9*w; tmpy = h/2; tmpColor = new Scalar(255,0,0);}
    		}
    		if (ok>0) Core.circle(mRgba, new Point(tmpx,tmpy), h/20, tmpColor, -1);
        	getLocation();
        	if (inputMode<1) if (debugMode>0) Core.putText(mRgba, "a: "+azimuth+"/"+azimuthValid+"/"+azimuthLimit+"/"+mArea+"/"+minArea, new Point(4,(h/4)), 1, siz, new Scalar(255,255,150), wi);
        	if (inputMode<1) if (debugMode>0) Core.putText(mRgba, "g: "+stav+"/"+Math.round(accuracy)+"/"+Math.round(bearing)+"/"+Math.round(100000*lat)+"/"+Math.round(100000*lon), new Point(4,(3*h/8)), 1, siz, new Scalar(255,255,150), wi);
        	computePosition(); // compute actual position (estimation, latOK, lonOK, azimuthOK, distanceOK)
        	int[] edge = {0,0,0,0};
        	if (edges.size()>0)  edge = edges.get(0);
        	if (inputMode<1) {
        		if (debugMode>0) Core.putText(mRgba, "p: "+path.size()+"/"+wp+"/"+wpMode+"/"+Math.round(wpDist)+"/"+Math.round(wpAzim0)+"/"+Math.round(1000*wpLat)/1000+"/"+Math.round(1000*wpLon)/1000, new Point(corner,(h/2)), 1, siz, new Scalar(255,255,150), wi);
            	if (goals.size()>1) if (debugMode>0) Core.putText(mRgba, "t: "+goals.size()+"/"+Math.round(1000*goals.get(0).x)/1000+"/"+Math.round(1000*goals.get(0).y)/1000+"/"+Math.round(1000*goals.get(1).x)/1000+"/"+Math.round(1000*goals.get(1).y)/1000+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
            	else if (goals.size()>0) if (debugMode>0) Core.putText(mRgba, "t: "+goals.size()+"/"+Math.round(1000*goals.get(0).x)/1000+"/"+Math.round(1000*goals.get(0).y)/1000+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
            	else if (debugMode>0) Core.putText(mRgba, "t: "+goals.size()+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
        	}
        	double tmpx0 = 0;
        	double tmpy0 = 0;
        	tmpx = 0;
        	tmpy = 0;
        	// add map points to mRgba
            for (int i=0; i<points.size(); i++) {
            	tmpx = (points.get(i).x - centLon)*multLon+w/2;
            	tmpy = (centLat - points.get(i).y)*multLat+h/2;
        		Core.circle(mRgba, new Point(tmpx,tmpy), 4, new Scalar(0,0,255),-1);
            }
        	// add edges to mRgba
            for (int i=0; i<edges.size(); i++) {
            	edge = edges.get(i);
            	int p1 = edge[0];
            	int p2 = edge[1];
            	tmpx0 = (points.get(p1).x - centLon)*multLon+w/2;
            	tmpy0 = (centLat - points.get(p1).y)*multLat+h/2;
            	tmpx = (points.get(p2).x - centLon)*multLon+w/2;
            	tmpy = (centLat - points.get(p2).y)*multLat+h/2;
        		Core.line(mRgba, new Point(tmpx0,tmpy0), new Point(tmpx,tmpy), new Scalar(0,0,255),2);
            }
        	// add goals to mRgba
            for (int i=0; i<goals.size(); i++) {
            	tmpx = (goals.get(i).x - centLon)*multLon+w/2;
            	tmpy = (centLat - goals.get(i).y)*multLat+h/2;
        		Core.circle(mRgba, new Point(tmpx+3,tmpy+0), 4, new Scalar(255,0,0),-1);
            	Core.putText(mRgba, ""+i, new Point(tmpx,tmpy), 1, siz, new Scalar(255,255,255), wi);
            }
        	// add path to mRgba
            for (int i=0; i<path.size(); i++) {
            	tmpx = (path.get(i).x - centLon)*multLon+w/2+1;
            	tmpy = (centLat - path.get(i).y)*multLat+h/2+1;
            	if (i==wp) Core.circle(mRgba, new Point(tmpx-3,tmpy-0), 10, new Scalar(200,200,200),-1);
            	if (wpModes.get(i)[0]>0) Core.circle(mRgba, new Point(tmpx-3,tmpy-0), 5, new Scalar(255,153,0),-1);
            	else Core.circle(mRgba, new Point(tmpx-3,tmpy-0), 4, new Scalar(0,255,0),-1);
        		if (i>0) Core.line(mRgba, new Point(tmpx0,tmpy0), new Point(tmpx,tmpy), new Scalar(0,255,0),1);
        		tmpx0 = tmpx;
        		tmpy0 = tmpy;
            }
        	// add actual GPS position to mRgba
            if (stav=="OK") {
            	tmpx = (lon - centLon)*multLon+w/2;
            	tmpy = (centLat - lat)*multLat+h/2;
        		Core.circle(mRgba, new Point(tmpx,tmpy), 6, new Scalar(255,0,0),-1);
            }
        	mColor = RED; tx = "I";
        	if (inputMode>0) {mColor = GREEN; tx = "I";}
        	Core.rectangle(mRgba, new Point(0,0), new Point(corner,corner), mColor, 1);
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
        	Core.rectangle(mRgba, pt5, pt6, mColor1, -1);
        	Core.rectangle(mRgba, pt7, pt8, mColor2, -1);
        	if (mod2>0) {
        		mColor1 = BLUE;
        		azimLimit = 30;
        	} else {
        		mColor1 = BLACK;
        		azimLimit = 20;
        	}
        	if (mod6<=1) mColor2 = BLACK;
        	else if (mod6<=3) mColor2 = BLUE;
        	else if (mod6<=5) mColor2 = GREEN;
        	else mColor2 = YELLOW;
        	tx = "+";
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,h/2-corner2-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,0,0), 2*wi);
        	tx = "-";
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,h/2+corner2+corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,0,0), 2*wi);
        	now.setToNow();
        	tx = now.format("%H:%M:%S");
        	tx += " ("+startTime+")";
        	Core.putText(mRgba, tx, new Point(w/4,h/25+2), 1, siz, new Scalar(255,255,100), wi);
        }
        if (!stopped) {
        	mColor = RED; tx = "D";
        	if (debugMode>0) {mColor = GREEN; tx = "D";}
        	Core.rectangle(mRgba, new Point(0,h), new Point(corner,h-corner), mColor, 1);
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,h-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
        	mColor = RED; tx = "M";
        	if (voiceOutput>0) {mColor = GREEN; tx = "V";}
        	Core.rectangle(mRgba, pt1, pt2, mColor, -1);
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
        	mColor = WHITE;
        	if (searchMode>0) mColor = BLACK;
        	if (searchMode==1) mColor = BLUE;
        	else if (searchMode==2) mColor = YELLOW;
        	else if (searchMode==3) mColor = GREEN;
        	else if (searchMode==4) mColor = RED;
        	if (inputMode<1) Core.rectangle(mRgba, pt3, pt4, mColor, -1);
        	tx = ""+searchMode+sm[searchMode];
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(w-corner-textSize.width/2+1,h-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(200,200,200), 2*wi);
        	Core.rectangle(mRgba, new Point(0,h/2+corner/2), new Point(corner,h/2-corner/2), BLACK, -1);
        	tx = "S";
        	textSize = Core.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Core.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,h/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,255,255), 2*wi);
        	directionNum++;
            directionTrend += direction;
            topPointTrend += topDirection;
        }
        return mRgba;
    }

    private Scalar converScalarHsv2Rgba(Scalar hsvColor) {
        Mat pointMatRgba = new Mat();
        Mat pointMatHsv = new Mat(1, 1, CvType.CV_8UC3, hsvColor);
        Imgproc.cvtColor(pointMatHsv, pointMatRgba, Imgproc.COLOR_HSV2RGB_FULL, 4);

        return new Scalar(pointMatRgba.get(0, 0));
    }
    
    static void setToRgba(String addToRgba) {
    	tmpText += addToRgba;
    	if (tmpText.indexOf("\n")>0) {
            try {
            	toRgba = tmpText.substring(0, tmpText.indexOf("\n")-1);
            	if (toRgba.length()>33) {
            		int tmp = Integer.parseInt(toRgba.substring(10,15).trim());
                	//if (tmp>0) initialAzimuth = tmp;
                	tmp = Integer.parseInt(toRgba.substring(6,10).trim());
                	if (tmp>0) {
                		azimuth = tmp;
                		now.setToNow();
                		azimuthValidTime = now.toMillis(false) + 3000;
                		azimuthValid = 1;
                		btValid = 1;
                    	//azimuthDifference = azimuth - initialAzimuth;
                    	azimuthDifference = (int)(azimuth - wpAzim1);
                    	if (azimuthDifference<-180) azimuthDifference += 360;
                    	else if (azimuthDifference>180) azimuthDifference -= 360;
                    	btPwm = Integer.parseInt(toRgba.substring(15,19).trim());
                    	btRng = Integer.parseInt(toRgba.substring(23,27).trim());
                    	btDst = Integer.parseInt(toRgba.substring(27,31).trim());
                    	btSpd = Integer.parseInt(toRgba.substring(31,35).trim());
                	}
            		//String[] tmpStr = tmpText.split("\t| ");
                	if (toRgba.length()>54) {
                		btLat = Float.parseFloat(toRgba.substring(39,48).trim());
                		btLon = Float.parseFloat(toRgba.substring(48,57).trim());
                		btRngLeft = Integer.parseInt(toRgba.substring(57,61).trim());
                		btRngRight = Integer.parseInt(toRgba.substring(61,65).trim());
                		btRngBack = Integer.parseInt(toRgba.substring(65,69).trim());
                    	btPwm = Integer.parseInt(toRgba.substring(69,73).trim());
                		//btLat = Float.parseFloat(tmpStr[9].trim());
                		//btLon = Float.parseFloat(tmpStr[10].trim());
                		//btRngLeft = Integer.parseInt(tmpStr[11].trim());
                		//btRngRight = Integer.parseInt(tmpStr[12].trim());
                		//btRngBack = Integer.parseInt(tmpStr[13].trim());
                	}
            	}
                //azimuth = 88;
                //initialAzimuth = 99;
        		tmpText = tmpText.substring(tmpText.indexOf("\n")+1);
            } catch (NumberFormatException e) {
                //azimuth = 99;
                //initialAzimuth = 88;
            }
        	//tmpText = "";
    	}
    }

    static void setSearchMode(int sMode) {
    	searchMode = sMode;
    }

    static void setLevel(int sLimit) {
    	mLevel = sLimit;
    }

    static void setArea(int sArea) {
    	mArea = sArea;
    	//minArea = ((double)sArea)/100;
    	minArea = 0.5;
    }

    static void setHSV1(String sHSV1) {
    	mHSV1 = sHSV1;
    }

    static void setHSV2(String sHSV2) {
    	mHSV2 = sHSV2;
    }

    static void setVoiceOutput(int sVoice) {
    	voiceOutput = sVoice;
    }

    public static void setOrientation(int sOrientation) {
        mOrientation = sOrientation;
    }

    static void setAddress(String sAddress) {
    	address = sAddress;
    }

    static void setAddress2(String sAddress2) {
    	address2 = sAddress2;
    }
    
    static void setStartTime() {
    	now.set(startTimeMilis);
    	startTime = now.format("%H:%M:%S");
    	now.setToNow();
    }

    private void sendCommand() {
        final Timer timer = new Timer();
        timer.schedule(new TimerTask() {

        @Override
        public void run() {
          	if (stopped) {
                timer.cancel();
                timer.purge();
     		}
          	if (!stopped && searchMode>0) {
               	txtCommand = "";
                if (stav=="OK" && lat>10f && inputMode<1 && wpDist1<3000) {
                	centLat = lat;
                	centLon = lon;
                    //Toast.makeText(getApplicationContext(), "center", Toast.LENGTH_SHORT).show();
                }
          		// countdown
          		now.setToNow();
          		long tmpTime = now.toMillis(false);
      			tmpSec = (startTimeMilis - tmpTime)/1000;
          		if (runMode<1) {
          			// countdown active
          	    	String txtCommand1 = "";
          	    	if (tmpSec==120) txtCommand1 = "2 minutes";
          	    	else if (tmpSec==60) txtCommand1 = "1 minute";
          	    	else if (tmpSec==50) {
          	    		if (stav=="OK") txtCommand1 = "GPS ready";
          	    		else txtCommand1 = "GPS malfunction";
          	    	}
          	    	else if (tmpSec==40) {
          	    		if (azimuthValid>0) txtCommand1 = "compass ready";
          	    		else txtCommand1 = "bluetooth malfunction";
          	    	}
          	    	else if (tmpSec==30) txtCommand1 = "30 seconds";
          	    	else if (tmpSec==20) txtCommand1 = "20 seconds";
          	    	else if (tmpSec==10) txtCommand1 = "10 seconds";
          	    	else if (tmpSec==0) txtCommand1 = "go";
          	    	else if (tmpSec<=5 && tmpSec>0) txtCommand1 = ""+(int)tmpSec;
      		    	if (txtCommand1!="" && !txtCommand1.equals(pom) && voiceOutput>0) {
          		    	pom = txtCommand1;
          		    	tts.speak(txtCommand1, TextToSpeech.QUEUE_ADD, null);
      		    	}
      		    	return;
          		}
          		computeCommand(); // main navigation algorithm => mCommand
          		if (searchMode>1 && tmpSec>=-5) mCommand = 'w'; // initial mCommand is "forward"
    	    	out[0] = mCommand;
             	txtCommand = "";
                if (mCommand!='-') {
                 	writeCommand();
                	tellCommand((char)out[0]);
                 	mPrevCommand = mCommand;
                 	mCommand = '-';
                } else if (mPrevCommand!='-') {
                	// send 'f' once before nothing ('-')
        	    	out[0] = 'f';
                 	writeCommand();
                	tellCommand((char)out[0]);
                	mPrevCommand = '-';
                }
            	SimpleDateFormat format = new SimpleDateFormat("yyMMdd_HHmmss",Locale.US);
            	String dateTimeString = format.format(new Date());
                //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+direction+";"+topDirection+";"+averageDirection+";"+averageTopPoint+";"+directionNum+";"+azimuth+";"+initialAzimuth+";"+azimuthDifference+";"+azimuthValid+";"+btPwm+";"+btRng+";"+btDst+";"+btSpd+";"+limit1+";"+limit2+";"+mLevel+";"+azimuthLimit+";"+mArea+";"+minArea+";"+btLat+";"+btLon+";"+btRngLeft+";"+btRngRight+";"+btRngBack+";");
            	// new log structure (4.9.2015)
                //appendLog("date_time;mCommand;searchMode;direction;topDirection;cameraDirection;directionNum;azimuth;azimuthValid;btPwm;btRng;btRngLeft;btRngRight;btRngBack;btDst;btSpd;limit1;limit2;mLevel;azimuthLimit;mArea;minArea;btLat;btLon;stav;lat;lon;bearing;accuracy;wp;wpLat;wpLon;wpMode;wpDist;wpAzim0;wpDist1;wpAzim1;azimuth;actualDist;lastDist;");
                appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+direction+";"+topDirection+";"+cameraDirection+";"+azimuth+";"+azimuthValid+";"+btPwm+";"+btRng+";"+btRngLeft+";"+btRngRight+";"+btRngBack+";"+btDst+";"+btSpd+";"+limit1+";"+limit2+";"+mLevel+";"+azimuthLimit+";"+mArea+";"+minArea+";"+Math.round(100000*btLat)+";"+Math.round(100000*btLon)+";"+stav+";"+Math.round(100000*lat)+";"+Math.round(100000*lon)+";"+Math.round(bearing)+";"+Math.round(accuracy)+";"+wp+";"+Math.round(100000*wpLat)+";"+Math.round(100000*wpLon)+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(wpAzim0)+";"+Math.round(wpDist1)+";"+Math.round(wpAzim1)+";"+azimuth+";"+Math.round(azimDiff)+";"+Math.round(azimDiff1)+";"+Math.round(actualDist)+";"+Math.round(lastDist)+";");
                directionTrend = -direction/2;
                topPointTrend = -topDirection/2;
                directionNum = 0;
          	}
          }
        },0,990); //run every defined ms
    }
    
    private void writeCommand() {
  		out1[0] = out[0];
  		if ((char)out[0]>='a') {
  	  	    out1[0] = (byte)(commandsMap.get((char)out[0]) & 0xFF);
  		}
     	MainActivity.mSerialService.write(out1);
    }
    
    private void tellCommand(char command) {
    	txtCommand = "";
    	if (command=='l') txtCommand = "xleft";
    	else if (command=='r') txtCommand = "xright";
    	else if (command=='h') txtCommand = "left";
    	else if (command=='k') txtCommand = "right";
    	else if (command=='f') txtCommand = "straigt";
    	else if (command=='s') txtCommand = "stop";
    	else if (command=='b') txtCommand = "back";
    	else if (command=='w') txtCommand = "forward";
    	else if (command=='p') txtCommand = "drop";
    	else if (command=='t') txtCommand = "turn";
    	else if (command=='$') txtCommand = "finish";
    	else if (command>='0' && command<='9') txtCommand = ""+command;
    	if (txtCommand!="") {
    		if (searchMode!=1 || mCommand!='w' || mPrevCommand!=mCommand) {
            	if (voiceOutput>0) tts.speak(txtCommand, TextToSpeech.QUEUE_ADD, null);
        		//Toast.makeText(getApplicationContext(), txtCommand, Toast.LENGTH_SHORT).show();
    		}
    	}
    }
    
    private void saveConfig() {
        Toast.makeText(getApplicationContext(), "saving...", Toast.LENGTH_SHORT).show();
        mPrefs = PreferenceManager.getDefaultSharedPreferences(this);
    	SharedPreferences.Editor editor = mPrefs.edit();
    	editor.putString(SEARCHMODE_KEY, Integer.toString(searchMode));
    	editor.putString("btdevice", address);
    	editor.putString("btdevice2", address2);
    	editor.putString("limit1", ""+limit1);
    	//editor.putString("minarea", ""+mArea);
    	if (searchMode==0) editor.putString("hsv1", mHSV1);
    	else if  (searchMode==2) editor.putString("hsv2", mHSV2);
    	else editor.putString(THRESHOLDLIMIT_KEY, Integer.toString(mLevel));
    	editor.commit();
    }

    private void readPrefs() {
        mPrefs = PreferenceManager.getDefaultSharedPreferences(this);
		limit1 = readIntPref("limit1", limit1, 5);
		limit2 = limit1 + 10;
		azimuthLimit = readIntPref("azimlimit", azimuthLimit, 25);
		mArea = readIntPref("minarea", mArea, 9999);
		//if (mArea>10) minArea = mArea/100; 
		mHSV1 = mPrefs.getString("hsv1", mHSV1);
		mHSV2 = mPrefs.getString("hsv2", mHSV2);
		address = mPrefs.getString("btdevice", address);
		//if (address.length()<1) address = "00:00:00:00:00:00";
		address2 = mPrefs.getString("btdevice2", address2);
		commandsTable = mPrefs.getString("commands", commandsTable);
		if (commandsTable.length()<10) commandsTable = commandsTable0;
		commandsMap.clear();
		for (int ix=0; ix<commandsTable0.length(); ix++) {
			commandsMap.put(commandsTable0.charAt(ix), commandsTable.charAt(ix));
		}
   }

    private int readIntPref(String key, int defaultValue, int maxValue) {
        int val;
        try {
            val = Integer.parseInt( mPrefs.getString(key, Integer.toString(defaultValue)) );
        } catch (NumberFormatException e) {
            val = defaultValue;
        }
        val = Math.max(0, Math.min(val, maxValue));
        return val;
    }

    public void appendLog(String text)
    {
    	if (!stopped) {
    	       File sdCard = Environment.getExternalStorageDirectory();
    	       SimpleDateFormat format = new SimpleDateFormat("yyMMdd",Locale.US);
    	       String dateString = format.format(new Date());
    	   	   String fileName = sdCard.getAbsolutePath() + "/RoboNav" + dateString + ".log";
    	       File logFile = new File(fileName);
    	       if (!logFile.exists())
    	       {
    	          try
    	          {
    	             logFile.createNewFile();
    	          } 
    	          catch (IOException e)
    	          {
    	             // TODO Auto-generated catch block
    	             e.printStackTrace();
    	          }
    	       }
    	       try
    	       {
    	          //BufferedWriter for performance, true to set append to file flag
    	          BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
    	          buf.append(text);
    	          buf.newLine();
    	          buf.close();
    	       }
    	       catch (IOException e)
    	       {
    	          // TODO Auto-generated catch block
    	          e.printStackTrace();
    	       }
    	}
    }

    public void writeToFile(String file, String text)
    {
    	if (!stopped) {
    	       File sdCard = Environment.getExternalStorageDirectory();
    	   	   String fileName = sdCard.getAbsolutePath() + "/" + file;
    	       File logFile = new File(fileName);
    	       if (!logFile.exists())
    	       {
    	          try
    	          {
    	             logFile.createNewFile();
    	          } 
    	          catch (IOException e)
    	          {
    	             e.printStackTrace();
    	          }
    	       }
    	       try
    	       {
    	          //BufferedWriter for performance, true to set append to file flag
    	          BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, false));
    	          buf.write(text);
    	          //buf.newLine();
    	          buf.close();
    	       }
    	       catch (IOException e)
    	       {
    	          e.printStackTrace();
    	       }
    	}
    }

    public void getLocation()
    {
     // Get the location manager
     LocationManager locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
     //Criteria criteria = new Criteria();
     //String bestProvider = locationManager.getBestProvider(criteria, false);
     Location location = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
     try {
       lat = location.getLatitude();
       lon = location.getLongitude();
       accuracy = location.getAccuracy();
       bearing = location.getBearing();
       stav = "OK";
       //return new LatLng(lat, lon);
     }
     catch (NullPointerException e){
         e.printStackTrace();
         lat = -1.0;
         lon = -1.0;
         accuracy = 999;
         bearing = 999;
         //stav = e.toString();
         //stav = "ERR"+e.hashCode();
         stav = "ERR";
       //return null;
     }
    }

    public List<String> readFile(String fil)
    {
    	List<String> ret = new ArrayList<String>();
        try {
            File sdcard = Environment.getExternalStorageDirectory();
            File file = new File(sdcard,fil);
            BufferedReader br = new BufferedReader(new FileReader(file));  
            String line;   
            while ((line = br.readLine()) != null) {
            	if (line.length()>10) {
                	ret.add(line);
            	}
            }
            br.close();
        }
        catch (Exception e) {
            e.printStackTrace();                    
        }
        return ret;
    }

    public void computeNextWaypoint(int step)
    {
    	if (path.size()>(wp+step) && (wp+step)>=0) {
    		try {
        		// path = Point(lon,lat);
        		double wpLat0 = path.get(wp).y;
        		double wpLon0 = path.get(wp).x;
        		wp += step;
        		wpLat = path.get(wp).y;
        		wpLon = path.get(wp).x;
        		wpMode = wpModes.get(wp)[0];
        		float[] results = new float[3];
        		Location.distanceBetween(wpLat0, wpLon0, wpLat, wpLon, results);
        		wpDist = results[0];
        		wpAzim0 = results[1];
        		//wpAzim1 = results[2];
            	lastDist = btDst;
    		}
            catch (Exception e) {
                e.printStackTrace();                    
            }
    	}
    }

    public List<Point> readPoints(String fil)
    {
    	List<Point> ret = new ArrayList<Point>();
		if (fil.contains("Path")) wpModes.clear();;
        try {
            File sdcard = Environment.getExternalStorageDirectory();
            File file = new File(sdcard,fil);
            BufferedReader br = new BufferedReader(new FileReader(file));  
            String line;
    		double xLat = 0.0;
    		double xLon = 0.0;
            while ((line = br.readLine()) != null) {
            	if (line.length()>10) {
            		String[] tmp = line.split("\t| ");
            		xLat = Double.parseDouble(tmp[1]);
            		xLon = Double.parseDouble(tmp[2]);
            		int xMode0 = 0;
            		if (tmp[3].length()>0) xMode0 = Integer.parseInt(tmp[3]);
            		int[] xMode = {xMode0};
            		if (fil.contains("Path")) wpModes.add(xMode);
            		Point tmpPoint = new Point(xLon,xLat);
                	ret.add(tmpPoint);
            	}
            }
            br.close();
        }
        catch (Exception e) {
            e.printStackTrace();                    
        }
        return ret;
    }

    public void readMap(String fil)
    {
        //Toast.makeText(getApplicationContext(), "readMap "+fil, Toast.LENGTH_LONG).show();
        try {
            File sdcard = Environment.getExternalStorageDirectory();
            File file = new File(sdcard,fil);
            BufferedReader br = new BufferedReader(new FileReader(file));  
            String line;
            points.clear();
            edges.clear();
        	minLat = 999.0;
        	minLon = 999.0;
        	maxLat = -999.0;
        	maxLon = -999.0;
            double xLat = 0.0;
    		double xLon = 0.0;
            while ((line = br.readLine()) != null) {
            	if (line.length()>2) {
            		String[] tmp = line.split("\t| ");
            		//test += "."+tmp.length;
            		if (tmp.length>2) {
            			// point
                		xLat = Double.parseDouble(tmp[1]);
                		xLon = Double.parseDouble(tmp[2]);
                		//int xMode = Integer.parseInt(tmp[3]);
                		Point tmpPoint = new Point(xLon,xLat);
                    	points.add(tmpPoint);
                    	if (xLat<minLat) minLat = xLat;
                    	else if (xLat>maxLat) maxLat = xLat;
                    	if (xLon<minLon) minLon = xLon;
                    	else if (xLon>maxLon) maxLon = xLon;
            		} else if (tmp.length==2) {
            			// edge
            			int p1 = Integer.parseInt(tmp[0]);
            			int p2 = Integer.parseInt(tmp[1]);
            			double lat0 = points.get(p1).y;
            			double lon0 = points.get(p1).x;
            			double lat1 = points.get(p2).y;
            			double lon1 = points.get(p2).x;
                		float[] results = new float[3];
                		Location.distanceBetween(lat0, lon0, lat1, lon1, results);
                		int dist = Math.round(results[0]);
                		int azim = Math.round(results[1]);
                		int[] edge = {p1,p2,dist,azim};
            			edges.add(edge);
            		}
            	}
            }
            br.close();
            centLat = (maxLat + minLat)/2;
            centLon = (maxLon + minLon)/2;
            //multLat = h/0.0054;
            //multLon = w/0.0087;
            multLat = 2*111111.1;
            multLon = 2*70000.0;
            //if ((maxLat - minLat)>0) multLat = 0.7*h/(maxLat - minLat); 
            //if ((maxLon - minLon)>0) multLon = 0.7*w/(maxLon - minLon); 
        }
        catch (Exception e) {
            e.printStackTrace();                    
        }
        //Toast.makeText(getApplicationContext(), ""+centLat+" "+centLon+" "+minLat+" "+minLon+" "+maxLat+" "+maxLon, Toast.LENGTH_LONG).show();
    }

    public void computePosition()
    {
    	compassAzimuth = compass.getAzimuth();
    	latOK = lat;
    	lonOK = lon;
		azimuthValid = 1;
    	if (btValid>0) {
        	azimuthOK = azimuth;
        	distanceOK = btDst;
        	if (btLat>0f) {
        		// GPS at robot is better
            	latOK = btLat;
            	lonOK = btLon;
        	}
    	} else if (compassAzimuth!=999) {
    		azimuthOK = compassAzimuth;
    	} else if (stav=="OK") {
    		// azimuth from GPS?
    		if (bearing!=0) {
            	azimuthOK = (bearing + 360) % 360;
    		}
    	} else {
    		// azimuth from path
        	azimuthOK = wpAzim0;
    		azimuthValid = 0;
    	}
    	if (azimuthOK>180) azimuthOK -= 360;
    }
    
    @SuppressLint("FloatMath")
	private float spacing(MotionEvent event) {

        float x = event.getX(0) - event.getX(1);
        float y = event.getY(0) - event.getY(1);
        return FloatMath.sqrt(x * x + y * y);
    }

    public void init() {
    	btPwm = 0; // PWM from bluetooth
    	btRng = 99; // center sonar range from bluetooth
    	btDst = 0; // distance from bluetooth
    	btSpd = 0; // speed from bluetooth
    	btRngLeft = 99; // left sonar range from bluetooth
    	btRngRight = 99; // right sonar range from bluetooth
    	btRngBack = 99; // IR range from bluetooth
    	wp = 0; // waypoint number
    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	goalReached = 0; // we should be at start now
    	state = "normal";
        appendLog("init");
}

    public void computeCommand()
    {
    	// main navigation algorithm (finite automata and fuzzy logic)
        mod2 = mLevel % 2;
        mod6 = mLevel % 6;
		float[] results = new float[3];
		Location.distanceBetween(latOK, lonOK, wpLat, wpLon, results);
		wpDistOK = results[0];
		wpAzim1 = results[1];
    	azimDiff1 = wpAzim0 - wpAzim1; // actual azimuth difference
    	if (azimDiff1<-180) azimDiff1 += 360;
    	else if (azimDiff1>180) azimDiff1 -= 360;
    	averageDirection = direction;
    	averageTopPoint = topDirection;
    	if (directionNum>0) {
    		averageDirection = (int)Math.round(directionTrend/directionNum);
    		averageTopPoint = (int)Math.round(topPointTrend/directionNum);
    	}
    	btValid = 0;
    	now.setToNow();
    	if (now.toMillis(false)<azimuthValidTime) btValid = 1;
    	else btValid = 0; // bleatooth data are not valid
    	/* signals for navigation:
    	 * - averageDirection (direction to center of the road blob)
    	 * - averageTopPoint (direction to top point of the road blob)
    	 * - topHeight (how far is the topPoint from the base) {0 to 8}
    	 * - azimuthOK (actual azimuth of the robot)
    	 * - azimDiff (actual azimuth difference to the line between the last and next waypoint)
    	 * - azimDiff1 (actual azimuth difference to the line between the actual position and the next waypoint)
    	 * - actualDist (actual distance from the last waypoint)
    	 * - wpDist (distance from last waypoint to the next waypoint)
    	 * - btRng, btRngLeft, btRngRight (ranges to obstacles ahead, left and right)
    	 * - btValid (1 => data from BT link are valid, 0 = BT data are not valid)
    	 * - azimuthValid (1 => measured azimuth is valid, 0 = azimuth is not measured)
    	 * - state (actual state of the algorithm)
    	 * - wpDistOK (distance to the next waypoint)
    	 * - wp (number of the next waypoint)
    	 * - cameraDirection (mean direction between averageDirection and averageTopPoint)
    	 * - cameraDiff (difference between averageDirection and averageTopPoint)
    	 * - cameraProbability (probability of the accuracy of the cameraDirection)
    	 * - azimuthLimit, limit1, limit2 (limits of the azimuth difference)
    	 */
		mCommand = '-'; // implicit mCommand
		if (searchMode>0) {
			// finite automata navigation
	  		//mCommand = '-'; // implicit mCommand
  			actualDist = distanceOK - lastDist;
  			// obstacle avoiding
  			if (btRng<12 || btRngLeft<8 || btRngRight<8) {
  				if (avoiding<1) {
  		    		mCommand = 's'; // stop
  		    		avoiding = 1;
  		    		return;
  				} else if (btRngBack>30) {
  		    		mCommand = 'b'; // back
  		    		return;
  				} else {
  		    		mCommand = 's'; // stop
  		    		return;
  				}
  			} else if (avoiding>0) {
	    		avoiding = 0;
	    		mCommand = 'w'; // forward
	    		return;
  			}
  			// step backward if state was unknown;
  			if (state=="unknown") {
  				state = "back";
  				mCommand = 'b';
  				return;
  			}
  			else if (state=="back") {
  				state = "back2";
  				mCommand = 'b';
  				return;
  			}
  			else if (state=="back2") {
  				state = "back3";
  				mCommand = 's';
  				return;
  			}
  			else if (state=="back3") {
  				state = "normal";
  				mCommand = 'w';
  				return;
  			}
	  		// waypoint navigation
	  		if (wpDist>0 && wpDist<999 && searchMode>1) {
	  			if (actualDist>(wpDist-2) || wpDistOK<5 || (wpDistOK<15 && Math.abs(azimDiff1)>90) || state=="drop") {
	  				// waypoint reached
	  		    	if (path.size()==(wp+1) || wp==0 || wpModes.get(wp)[0]>0 || state=="drop") {
	  		    		// payload drop
	  		    		if (tmpSec0==0) {
	  		    			// start payload drop procedure
	  		    			state = "drop";
	  		    			tmpSec0 = tmpSec;
		  		    		mCommand = 's'; // stop
		  		    		counter = 0;
		  		    		return;
	  		    		} else if ((tmpSec0-tmpSec)>9) {
	  		    			// end of payload drop procedure
	  		    			if (goalReached>0) {
		  		    			if (path.size()==(wp+1) || wp==0) {
		  		    				// end of mission
			  		    			state = "end";
		  		    				goalReached = 3; // end
				  		    		mCommand = 's'; // stop
				  		    		return;
		  		    			}
	  		    			}
	  		    			if (path.size()==(wp+1)) {
	  		    				goalReached = 2; // go to previous waypoint
	  			  				computeNextWaypoint(-1);
	  		    			} else {
		  		    			goalReached = 1; // go to next waypoint
				  				computeNextWaypoint(1);
	  		    			}
	  		    			state = "normal";
	  		    			tmpSec0 = 0;
		  		    		mCommand = 'w'; // forward
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec)%2)==1) {
	  		    			state = "drop";
		  		    		mCommand = 's'; // stop
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec)%2)==0) {
	  		    			state = "drop";
		  		    		mCommand = 'p'; // payload drop
		  		    		return;
	  		    		} else {
	  		    			state = "drop";
		  		    		mCommand = 's'; // stop
		  		    		return;
	  		    		}
	  		    	} else {
	  		    		// normal waypoint (without payload drop)
	  		    		if (goalReached!=2) computeNextWaypoint(1);
	  		    		else computeNextWaypoint(-1);
  		    			//state = "waypoint";
	  		    	}
	  			}
	  		}
  			if (searchMode>1 && (goalReached>=3 || state=="end")) {
  				// end of mission
    			state = "end";
		    	mCommand = 's'; // stop
		    	if (counter>0) {
		    		mCommand = '$'; // finish
		    		counter = 0;
		    	} else {
		    		counter = 1;
		    	}
		    	return;
  			}
  			// long range obstacle avoiding
  			if (btValid>0) {
  				if (btRng>19 && btRngLeft<19 && btRngRight>19) {
  					mCommand = 'k'; // slightly right
  				} else if (btRng>19 && btRngLeft>19 && btRngRight<19) {
  					mCommand = 'h'; // slightly left
  				}
  			}
  			// navigation to the next waypoint
			Location.distanceBetween(latOK, lonOK, wpLat, wpLon, results);
			wpDistOK = results[0];
			wpDist1 = wpDistOK;
			wpAzim1 = results[1];
	    	azimDiff = azimuthOK - wpAzim0; // initial azimuth difference
	    	if (mTargetAzimuth!=999 && searchMode==1) azimDiff = azimuthOK - mTargetAzimuth; // azimuth difference
	    	if (azimDiff<-180) azimDiff += 360;
	    	else if (azimDiff>180) azimDiff -= 360;
	    	azimDiff1 = azimuthOK - wpAzim1; // actual azimuth difference
	    	if (azimDiff1<-180) azimDiff1 += 360;
	    	else if (azimDiff1>180) azimDiff1 -= 360;
	    	if (wpDistOK<15) {
	    		// next waypoint is close
		    	// drive to waypoint (by azimuth of current defined path)
	        	if (azimuthValid>0) {
	      			if (azimDiff>azimuthLimit) mCommand = 'l'; // left
	      			if (azimDiff<-azimuthLimit) mCommand = 'r'; // right
	        	}
	    	} else {
	    		// next waypoint is far away
		    	// drive to waypoint (by azimuth from current position to next waypoint)
	        	if (azimuthValid>0) {
	      			if (azimDiff1>azimuthLimit) mCommand = 'l'; // left
	      			if (azimDiff1<-azimuthLimit) mCommand = 'r'; // right
	        	}
	    	}
	  		// probabilistic road navigation
	  		if (directionNum>0) {
	  			direction = averageDirection;
	  			topDirection = averageTopPoint;
	  		}
	  		if (btValid>0 && btDst<20) {
	  			direction /= 3-btDst/10;
	  			topDirection /= 3-btDst/10;
	  		}
	  		cameraDirection = (2*direction + 3*topDirection) / 5;
	  		cameraDiff = Math.abs(direction - 2*topDirection);
	  		cameraProbability = 100 - (2*cameraDiff);
	  		if (topHeight<2) cameraProbability = 0; // blob is too close, camera is not reliable
	  		azimDiffOK = (int)(-azimuthValid * azimDiff);
	  		if (searchMode==1) mCommand = '-'; // delete previously computed command (set default command to none for RR)
			if (mod6==0 || mod6==2 || mod6==3) {
  			    // navigation by topDirection
				//drivingSignal = cameraDirection;
				directionOK = topDirection;
			} else if (mod6==1 || mod6==4 || mod6==5) {
  			    // navigation by camera direction
				directionOK = cameraDirection;
				//drivingSignal = topDirection;
			} else {
				// navigation by camera and compass
	  			if (azimuthValid>0) {
	  				// traversibility check by compass
	  				if (Math.abs(azimDiffOK)<azimLimit) {
	  					directionOK = (int)Math.round(azimDiffOK * 0.2 + cameraProbability * cameraDirection * 0.008);
	  				}
	  				else if ((azimDiff>0 && cameraDirection>0) || (azimDiff<0 && cameraDirection<0)) {
	  					directionOK = (int)Math.round(azimDiffOK * 0.1 + cameraProbability * cameraDirection * 0.009);
	  				}
	  				else {
	  					directionOK = (int)Math.round(azimDiffOK * 0.8 + cameraProbability * cameraDirection * 0.002);
	  				}
	  			}
	  			else directionOK = (int)Math.round(cameraProbability * cameraDirection * 0.01);
			}
	  		if (mod6<3) {
	  			// PD navigation by computed driving signal
	  			drivingSignal = directionOK + 0.2 * (directionOK - lastDirectionOK);
	  			lastDirectionOK = directionOK;
                if (drivingSignal<-limit2) mCommand = 'h'; // extra left
	  			else if (drivingSignal<=-limit1) mCommand = 'h'; // slightly left
	  			else if (drivingSignal>limit2) mCommand = 'k'; // extra right
	  			else if (drivingSignal>=limit1) mCommand = 'k'; // slightly right
	  		} else {
	  			// navigation by logic
	  			drivingSignal = directionOK;
		  		if (cameraProbability>20) {
		  			// steer to center of the road
	  				if (drivingSignal>limit2) mCommand = 'k'; // extra right
	  				else if (drivingSignal<-limit2) mCommand = 'h'; // extra left
	  				else if (drivingSignal>limit1 && azimDiffOK>-limit1) mCommand = 'k'; // slightly right
	  				else if (drivingSignal<-limit1 && azimDiffOK<limit1) mCommand = 'h'; // slightly left
		  		}
	  			if (azimDiffOK>azimuthLimit && mRightOK>0) mCommand = 'r'; // extra right
	  			else if (azimDiffOK<-azimuthLimit && mLeftOK>0) mCommand = 'l'; // extra left
	  		}
  			if (" kr".indexOf(mCommand)>0 && mRightOK<1) mCommand = 'w'; // can't turn right
  			else if (" hl".indexOf(mCommand)>0 && mLeftOK<1) mCommand = 'w'; // can't turn left
	  		if (" wlhkr-".indexOf(mCommand)>0 && topHeight<2) {
  				// road border seems too close, rather stop and step back (not for RR)
	  			if (searchMode>1) {
	    			state = "unknown";
	    			mCommand = 's'; // stop
	  			}
	  		}
		}
      	if (mPrevCommand=='b' && mCommand!='b') {mCommand = 's'; state="stop";}	
    	if (mCommand=='b' && mPrevCommand!='b' && mPrevCommand!='s') mCommand = 's';
    	//if (mCommand=='b' && mPrevCommand=='s') mCommand = 'f';
		if (searchMode==1 && mCommand=='-') mCommand = 'w'; // normal mCommand for RR is "forward"
    	if ((" lhkr-".indexOf(""+mCommand)>0 && (state=="normal" && mPrevCommand=='s')) || state=="stop") mCommand = 'f'; // straight
    	if (" lhkr-".indexOf(""+mCommand)>0 && state=="normal" && btSpd<1 && btValid>0) mCommand = 'w'; // forward (if speed is 0)
    	if (mCommand=='w' && state=="stop") state = "normal";
		if (searchMode==1) {
			// stabilization for RR
			numCleanCommands++;
			if (mCommand!='w') {
				if (numCleanCommands<3) mCommand = 'w';
			    else {
			    	numCleanCommands = 0;
			    	if (btSpd>160) numCleanCommands = 1;
			    }
				
			}
		}
    }
}
