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
//import java.util.Vector;


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
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
//import org.opencv.features2d.KeyPoint;
import org.opencv.imgproc.Imgproc;

//import com.google.android.gms.common.ConnectionResult;
//import com.google.android.gms.common.GooglePlayServicesUtil;
//import com.google.android.gms.common.api.GoogleApiClient;
//import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
//import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
//import com.google.android.gms.location.LocationListener;
//import com.google.android.gms.location.LocationRequest;
//import com.google.android.gms.location.LocationServices;






import android.annotation.SuppressLint;
import android.app.Activity;
import android.content.ActivityNotFoundException;
import android.content.Context;
//import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
//import android.location.GpsStatus;
import android.os.Bundle;
import android.os.Environment;
//import android.os.Handler;
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

public class NavigationActivity extends Activity implements OnTouchListener, CvCameraViewListener2, LocationListener {
    private static final String  TAG              = "VisualNavigation::Activity";

	private static int tstText = 0;
	private static String tmpText = "";
	private static String toRgba = "none";
    private static final String SEARCHMODE_KEY = "searchmode";
    private static final String THRESHOLDLIMIT_KEY = "thresholdlimit";
	
	private TextToSpeech tts;
	
    private Mat                  mRgba;
    private Mat                  mTemp;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private Scalar               mBlobColorHsv1;
    private Scalar               mBlobColorHsv2;
    private ColorBlobDetector    mColorDetector;
    private RoadDetector         mRoadDetector;
    private ObstacleDetector         mObstacleDetector;
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
	private static Scalar VIOLET = new Scalar(255,0,255,255);
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
	private static String[] sm = new String[]{"MC","RR","RO","RT","RA","PC","LF","KH","BR","RC"};
	private static int mOrientation = 0;
	private static String mHSV1 = "";
	private static String mHSV2 = "";
    private static String address = "00:00:00:00:00:00";
    private static String address2 = "00:00:00:00:00:00";
    private static String allowedAddresses = "";
    private static String connectedAddress = "";
	private static String commandsTable0 = "lrswfbhkptnqe";
	private static String commandsTable = "lrswfbhkptnqe";
	private static String telemetryTable = "   ... hhh ...  ... ... fff ddd sss ... aaaaaaaa oooooooo lll rrr bbb www";
	private static Map<Character, Character> commandsMap = new HashMap<Character, Character>();
	private SharedPreferences mPrefs;
	private int direction = 0;
	private int blobDirection = 0;
	private int topDirection = 0;
    private int directionTrend = 0;
	private int directionNum = 0;
	private static int azimuth = 0;
	//private static int initialAzimuth = 0;
	private static int azimuthLimit = 25;
	private static int azimuthDifference = 0;
	private static Time now = new Time();
	private static long firstTime = now.toMillis(false);
	private static long azimuthValidTime = 0;
	private static int azimuthValid = 0;
	private static int btConnected = 0;
	private static int btValid = 0;
	private static int btPwm = 0; // PWM from bluetooth
	private static int btRng = 99; // center sonar range from bluetooth
	private static int btRngL = 99; // left center sonar range from bluetooth
	private static int btRngR = 99; // right center sonar range from bluetooth
	private static int btDst = 0; // distance from bluetooth
	private static int btSpd = 0; // speed from bluetooth
    private static double btLat = 0.0; // waypoint latitude from BT connection
    private static double btLon = 0.0; // waypoint longitude from BT connection
	private static int btRngLeft = 99; // left sonar range from bluetooth
	private static int btRngRight = 99; // right sonar range from bluetooth
	private static int btRngLeftL = 99; // left sonar range from bluetooth
	private static int btRngRightL = 99; // right sonar range from bluetooth
	private static int btRngLeftR = 99; // left sonar range from bluetooth
	private static int btRngRightR = 99; // right sonar range from bluetooth
	private static int btRngBack = 99; // IR range from bluetooth
	private static int btPayload = 0; // actual number of payloads
    private static boolean stopped = false;
    private static double lat = 0.0;
    private static double lon = 0.0;
    private static float accuracy = 888;
    private static float bearing = 888;
	private static String stav = "???";
    private static double latOK = 0.0; // best estimation of lattitude
    private static double lonOK = 0.0; // best estimation of longitude
    private static double azimuthOK = 0.0; // best estimation of azimuth (course)
    private static double azimuth_calib = 0.0; // calibration of azimuth (shift)
    private static int distanceOK = 0; // best estimation of distance (traveled)
	private static List<Point> path = new ArrayList<Point>(); // desired path
	private static int wp = 0; // waypoint number
    private static double wpLat = 0.0; // waypoint latitude
    private static double wpLon = 0.0; // waypoint longitude
    private static double wpDist = 0.0; // waypoint distance
    private static double pathAzimuth = 0.0; // azimuth from previous waypoint to next waypoint
    private static double azimuthToNextWaypoint = 0.0; // azimuth from actual GPS position to next waypoint
	private static int wpMode = 0; // waypoint mode (or points)
	private static int wpMode0 = wpMode;
    private static int lastDist = 0; // distance at last waypoint
	private static List<Point> goals = new ArrayList<Point>(); // waypoints
	private static List<Point> points = new ArrayList<Point>(); // map points
	private static List<int[]> edges = new ArrayList<int[]>(); // map edges (point1, point2, dist, azim)
	private static List<int[]> wpModes = new ArrayList<int[]>(); // waypoint modes (mode)
	private static List<int[]> goalPoints = new ArrayList<int[]>(); // goal points
	private static List<String> robots = new ArrayList<String>(); // robots (from RoboNavRobots.txt)
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
    private double turnAngleToNextWaypoint = 0;
    private int azimDiffOK = 0;
    private double distanceToNextWaypoint = 0;
    private double turnAngleToStayOnRoad = 0;
    private double turnAngleToAvoidObstacle = 0;
    private int move = 0;
	private long touchTime = 0;
	private long lastDownTime = 0;
	private long lastUpTime = 0;
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
	private String mTxt7 = "";
	private String mText = "";
	private int mTargetAzimuth = 999;
	private int compassAzimuth = 999;
	private int numCleanCommands = 0;
	private static double drivingSignal = 0.0;
	private int lastDirectionOK = 0;
	private int directionOK = 0;
	private int mLeftOK = 1;
	private int mRightOK = 1;
	private int mCenterOK = 1;
	private int mGrass = 0;
	private Point topPoint =  new Point(w/2,h/2);
	private Point centPoint =  new Point(w/2,h/2);
    //private List<KeyPoint> mObstacles = new ArrayList<KeyPoint>();
	
	private static int searchMode = 0;
	private static int mArea = 50;
	private static int voiceOutput = 1;
	private static int azimLimit = 20;

	private static int blobSearch = 0;
	private static int roadSearch = 1;

	private static long lastGPStime = -1;
	private static long lastNETtime = -1;

	private static int fromHeading = 0;
	private static int toHeading = 0;
	private static int fromSpeed = 0;
	private static int toSpeed = 0;
	private static int fromDistance = 0;
	private static int toDistance = 0;
	private static int fromPwm = 0;
	private static int toPwm = 0;
	private static int fromLat = 0;
	private static int toLat = 0;
	private static int fromLon = 0;
	private static int toLon = 0;
	private static int fromFront = 0;
	private static int toFront = 0;
	private static int fromLeft = 0;
	private static int toLeft = 0;
	private static int fromRight = 0;
	private static int toRight = 0;
	private static int fromBack = 0;
	private static int toBack = 0;
	private static int fromPayload = 0;
	private static int toPayload = 0;
	private static int repeatedCommand = 0;
	private static long lastSay = 0;
	private static long lastShow = 0;
	private static String navMode = "on";

	private static int fileNumber = 0;
	private String fileName = "";
	private String shortTxt = "";
	private static String testTxt = "";
   	private int shortNum = 0;
	private static String robotName = "robot";
	//String[] maps = {"t0","t1","t2","t3","0A","1A","1B","1C","1D","1E","1F","2A","2B","2C","2D","2E","2F","3A","3B","3C","3D","3E","3F"};
	String[] maps = {"t0","t1","t2","t3","t4","t5","t6","t7","t8","t9","0","A","B","C","D","E","F","G","H","CH","I","A2","B2","C2","D2","E2","F2","G2","H2","CH2","I2"};

	private static final int ACTIVITY_RESULT_QR_DRDROID = 0;
	private static String qrCode = "";

	private static Rect mBoundingRectangle = null;
	//private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
	private List<MatOfPoint> mContours = null;

	float[] results = new float[3];

	private LocationManager locationManager;
    
    //private GoogleApiClient mGoogleApiClient;

    private Compass compass;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    appendLog("NavigationActivity OpenCV "+OpenCVLoader.OPENCV_VERSION+" loaded OK");
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
	  	SimpleDateFormat format = new SimpleDateFormat("yyMMdd_HHmmss",Locale.US);
	  	String dateTimeString = format.format(new Date());
        appendLog(""+dateTimeString+" *** NavigationActivity instantiated");
    }

    void setLevel() {
    	mRoadDetector.setLevel(mLevel);
    	mObstacleDetector.setLevel(mLevel);
    	mColorDetector.setLevel(mLevel);
    }

	@Override
	public void onConfigurationChanged(Configuration newConfig) {
		super.onConfigurationChanged(newConfig);
		//Nothing
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

//        if (checkPlayServices()) {
//            startFusedLocation();
//            registerRequestUpdate(this);
//        }
//        if (mGoogleApiClient == null) {
//            mGoogleApiClient = new GoogleApiClient.Builder(this)
//                .addConnectionCallbacks(this)
//                .addOnConnectionFailedListener(this)
//                .addApi(LocationServices.API)
//                .build();
//        }
        
        locationManager = (LocationManager) this.getSystemService(Context.LOCATION_SERVICE);

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
        //appendLog("date_time;mCommand;searchMode;direction;topDirection;cameraDirection;azimuth;azimuthValid;btPwm;btRng;btRngLeft;btRngRight;btRngBack;btDst;btSpd;limit1;limit2;mLevel;azimuthLimit;mArea;minArea;btLat;btLon;stav;lat;lon;bearing;accuracy;wp;wpLat;wpLon;wpMode;wpDist;pathAzimuth;wpDist1;azimuthToNextWaypoint;azimuth;azimDiff;turnAngleToNextWaypoint;actualDist;lastDist;");
        //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+azimuthValid+";"+btDst+";"+btSpd+";"+Math.round(100000*lat)+";"+Math.round(100000*lon)+";"+Math.round(bearing)+";"+wp+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(pathAzimuth)+";"+Math.round(azimuthToNextWaypoint)+";");
    	// new log structure (27.11.2017)
        // appendLog("date_time;mCommand;searchMode;azimuthValid;btDst;btSpd;lat;lon;bearing;wpMode;wpDist;pathAzimuth;azimuthToNextWaypoint;");
    	// new log structure (5.12.2017)
        //appendLog("date_time;mCommand;searchMode;azimuthOK;btDst;btSpd;latOK;lonOK;bearing;wpMode;wpDist;pathAzimuth;azimuthToNextWaypoint;drivingSignal;");
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
        //compass.stop();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        String ocver = OpenCVLoader.OPENCV_VERSION_3_4_0;
        OpenCVLoader.initAsync(ocver, this, mLoaderCallback);
        appendLog("NavigationActivity OpenCV "+ocver+" loading initiated");
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
        stopped = false;
        appendLog("resume");
    }

    @Override
    protected void onStart() {
    	if (locationManager!=null) {
    		if (locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER))
    			locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);
    		if (locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER))
    			locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 0, 0, this);
    	}
        //mGoogleApiClient.connect();
    	compass.start();
		//Toast.makeText(getApplicationContext(), "compass start", Toast.LENGTH_SHORT).show();
    	stopped = false;
        super.onStart();
    }

    @Override
    public void onStop() {
        if (locationManager!=null) locationManager.removeUpdates(this);
        //stopFusedLocation();
    	super.onStop();
        appendLog("stop");
        compass.stop();
		//Toast.makeText(getApplicationContext(), "compass stop", Toast.LENGTH_SHORT).show();
    	stopped = true;
        //locationManager.removeUpdates(this);
        finish();
    }


    public void onDestroy() {
        //if (locationManager!=null) locationManager.removeUpdates(this);
        mRgba.release();
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        appendLog("destroy");
    	stopped = true;
        //locationManager.removeUpdates(this);
        if (mTemp!=null) mTemp.release();
        finish();
    }

//    private LocationRequest mLocationRequest;
//
//    private double fusedLatitude = 0.0;
//    private  double fusedLongitude = 0.0;
//
//    // check if google play services is installed on the device
//    private boolean checkPlayServices() {
//        int resultCode = GooglePlayServicesUtil
//                .isGooglePlayServicesAvailable(this);
//        if (resultCode != ConnectionResult.SUCCESS) {
//            if (GooglePlayServicesUtil.isUserRecoverableError(resultCode)) {
//                Toast.makeText(getApplicationContext(),
//                        "This device is supported. Please download google play services", Toast.LENGTH_LONG)
//                        .show();
//            } else {
//                Toast.makeText(getApplicationContext(),
//                        "This device is not supported.", Toast.LENGTH_LONG)
//                        .show();
//                finish();
//            }
//            return false;
//        }
//        return true;
//    }
//
//
//    public void startFusedLocation() {
//        if (mGoogleApiClient == null) {
//            mGoogleApiClient = new GoogleApiClient.Builder(this).addApi(LocationServices.API)
//                    .addConnectionCallbacks(new GoogleApiClient.ConnectionCallbacks() {
//                        @Override
//                        public void onConnectionSuspended(int cause) {
//                        }
//
//                        @Override
//                        public void onConnected(Bundle connectionHint) {
//
//                        }
//                    }).addOnConnectionFailedListener(new GoogleApiClient.OnConnectionFailedListener() {
//
//                        @Override
//                        public void onConnectionFailed(ConnectionResult result) {
//
//                        }
//                    }).build();
//            mGoogleApiClient.connect();
//        } else {
//            mGoogleApiClient.connect();
//        }
//    }
//
//    public void stopFusedLocation() {
//        if (mGoogleApiClient != null) {
//            mGoogleApiClient.disconnect();
//        }
//    }
//
//    public void registerRequestUpdate(final LocationListener listener) {
//        mLocationRequest = LocationRequest.create();
//        mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
//        mLocationRequest.setInterval(1000); // every second
//
//        new Handler().postDelayed(new Runnable() {
//            @Override
//            public void run() {
//                try {
//                    LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, listener);
//                } catch (SecurityException e) {
//                    e.printStackTrace();
//                } catch (Exception e) {
//                    e.printStackTrace();
//                    if (!isGoogleApiClientConnected()) {
//                        mGoogleApiClient.connect();
//                    }
//                    registerRequestUpdate(listener);
//                }
//            }
//        }, 1000);
//    }
//
//    public boolean isGoogleApiClientConnected() {
//        return mGoogleApiClient != null && mGoogleApiClient.isConnected();
//    }
//
//    @Override
//    public void onLocationChanged(Location location) {
//        setFusedLatitude(location.getLatitude());
//        setFusedLongitude(location.getLongitude());
//        
//    	SimpleDateFormat format = new SimpleDateFormat("yyMMdd_HHmmss",Locale.US);
//    	String dateTimeString = format.format(new Date());
//        appendToFile("RoboNavGps.log",dateTimeString+" "+fusedLatitude+" "+fusedLongitude);
//
//        //Toast.makeText(getApplicationContext(), "NEW LOCATION RECEIVED", Toast.LENGTH_LONG).show();
//
//    }
//
//    public void setFusedLatitude(double lat) {
//        fusedLatitude = lat;
//    }
//
//    public void setFusedLongitude(double lon) {
//        fusedLongitude = lon;
//    }
//
//    public double getFusedLatitude() {
//        return fusedLatitude;
//    }
//
//    public double getFusedLongitude() {
//        return fusedLongitude;
//    }

    
	@Override
	public void onLocationChanged(Location location) {
		if (location.getProvider()==LocationManager.GPS_PROVIDER) {
	        lat = location.getLatitude();
	        lon = location.getLongitude();
	        accuracy = location.getAccuracy();
	        bearing = location.getBearing();
	        stav = "OK";
	        lastGPStime = location.getTime();
		  	SimpleDateFormat format = new SimpleDateFormat("yyMMdd_HHmmss",Locale.US);
		  	String dateTimeString = format.format(new Date());
		    appendToFile("RoboNavGps.log",dateTimeString+" "+lat+" "+lon+" "+accuracy+" "+bearing);
	        //Toast.makeText(getApplicationContext(), "GPS location changed", Toast.LENGTH_SHORT).show();
		} else {
	        lat = location.getLatitude();
	        lon = location.getLongitude();
	        accuracy = location.getAccuracy();
	        bearing = location.getBearing();
	        stav = "NET";
	        lastNETtime = location.getTime();
		}
	}

	@Override
	public void onProviderDisabled(String arg0) {
        //Toast.makeText(getApplicationContext(), "GPS off", Toast.LENGTH_SHORT).show();
	}

	@Override
	public void onProviderEnabled(String arg0) {
        //Toast.makeText(getApplicationContext(), "GPS on", Toast.LENGTH_SHORT).show();
	}

	@Override
	public void onStatusChanged(String arg0, int arg1, Bundle arg2) {
        //Toast.makeText(getApplicationContext(), "GPS status changed", Toast.LENGTH_SHORT).show();
	}

    public void onCameraViewStarted(int width, int height) {
        //Toast.makeText(getApplicationContext(), "onCameraViewStarted", Toast.LENGTH_SHORT).show();
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mColorDetector = new ColorBlobDetector();
        mRoadDetector = new RoadDetector();
        mObstacleDetector = new ObstacleDetector();
        appendLog("cameraStart");
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
        mObstacleDetector.setHsvColor(mBlobColorHsv);
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
    	mText = "";
    	if (w>400) {
    		siz = 1.4;
    		wi = 2;
    	}
    	mBoundingRectangle = new Rect(w/2,h/2,1,1);
    	mRoadDetector.setLevel(mLevel);
    	mRoadDetector.setLimits(limit1,limit2);
    	mRoadDetector.setOrientation(mOrientation);
    	mColorDetector.setLevel(mLevel);
    	mColorDetector.setLimits(limit1,limit2);
    	mColorDetector.setOrientation(mOrientation);
        mColorDetector.setMinContourArea(minArea);
        mColorDetector.setMinArea(mArea);
        mColorDetector.setColorRadius(new Scalar(mArea/2,mArea,mArea,mArea));
    	mObstacleDetector.setLevel(mLevel);
    	mObstacleDetector.setLimits(limit1,limit2);
    	mObstacleDetector.setOrientation(mOrientation);
    	fileName = "RoboNavMap";
    	if (fileNumber>0) fileName += ""+fileNumber;
    	readMap(fileName+".txt");
    	fileName = "RoboNavGoals";
    	if (fileNumber>0) fileName += ""+fileNumber;
    	goals = readPoints(fileName+".txt");
    	fileName = "RoboNavPath";
    	if (fileNumber>0) fileName += ""+fileNumber;
    	path = readPoints(fileName+".txt");
    	//robots = readRobots("RoboNavRobots.txt");
    	//goals = path;
    	computeNextWaypoint(1);
    	now.setToNow();
    	startTimeMilis = now.toMillis(false)+300000;
    	startTimeMilis = (startTimeMilis/300000)*300000;
    	setStartTime();
    	runMode = 0; // wait for start time
		try {
			Intent trigger = getIntent();
			String sMode = trigger.getExtras().getString("sMode");
			String qrCode2 = trigger.getExtras().getString("qrCode");
			String sWp = trigger.getExtras().getString("sWp");
			String sState = trigger.getExtras().getString("sState");
            Toast.makeText(getApplicationContext(), "sMode: "+sMode+" / sWp: "+sWp+" / sState: "+sState+" / qrCode2: "+qrCode2, Toast.LENGTH_LONG).show();
			if (qrCode2.length()>0) {
				qrCode = qrCode2;
				appendLog("restart after QR Droid (qrCode: "+qrCode+")");
			}
			if (sMode.length()>0) {
				searchMode = Integer.parseInt(sMode);
				wp = Integer.parseInt(sWp);
				state = sState;
				computeNextWaypoint(0);
				now.setToNow();
				startTimeMilis = now.toMillis(false) - 10000;
				setStartTime();
				runMode = 1;
				appendLog("set new state (searchMode: "+searchMode+" / wp: "+wp+" / state: "+state+")");
			}
		} catch (Exception e) {
			// TODO
		}
        appendLog("date_time;mCommand;searchMode;azimuthOK;btDst;btSpd;latOK;lonOK;bearing;wpMode;wpDist;pathAzimuth;azimuthToNextWaypoint;drivingSignal;");
    }

    public void onCameraViewStopped() {
        appendLog("cameraStop");
        //finish();
    }

    public boolean onTouchEvent(MotionEvent event) {
    	//tts.speak("event", TextToSpeech.QUEUE_ADD, null);
    	
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        vw = mOpenCvCameraView.getWidth();
        vh = mOpenCvCameraView.getHeight();

        int offset = 40;
        if (rows<300) offset = 20;

        ex = (int)event.getX();
        ey = (int)event.getY();
        x = ex*cols/vw;
        y = ey*rows/vh-offset;

        switch (event.getAction() & MotionEvent.ACTION_MASK) {
		case MotionEvent.ACTION_DOWN:
        	//tts.speak("down", TextToSpeech.QUEUE_ADD, null);
			//move = 0;
            //lastUpTime = event.getEventTime();
			lastDownTime = event.getEventTime();
			touchMode = 1; // drag
			break;
		case MotionEvent.ACTION_UP:
        	//tts.speak("up", TextToSpeech.QUEUE_ADD, null);
			//touchTime = event.getEventTime() - event.getDownTime();
			touchTime = event.getEventTime() - lastDownTime;
    		//Toast.makeText(getApplicationContext(), ""+touchTime+" / "+event.getPressure(), Toast.LENGTH_SHORT).show();
    		//Toast.makeText(getApplicationContext(), ""+inputMode+" / "+move+" / "+touchMode+" / "+buttonTouched, Toast.LENGTH_SHORT).show();
            if (inputMode==1 && move==0 && buttonTouched==0 && (event.getEventTime()-lastDownTime)>=0 && (event.getEventTime()-lastUpTime)>=0) {
            	// new waypoint defined
        		//Toast.makeText(getApplicationContext(), "rows: "+rows+", vh: "+vh+" / "+cols+" / "+vw, Toast.LENGTH_SHORT).show();
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
            	double minDiff2 = 999.0;
            	int minIndex2 = 0;
                for (int i=0; i<goals.size(); i++) {
                	xLon = goals.get(i).x;
                	xLat = goals.get(i).y;
                	diff = Math.abs(tmpLon-xLon)+Math.abs(tmpLat-xLat);
                	if (diff<minDiff2) {
                		minDiff2 = diff;
                		minIndex2 = i;
                	}
                }
                if (minDiff<0.00015 || minDiff2<0.00015) {
                	if (minDiff<minDiff2) {
                		// waypoint is near of map point
                    	tmpLon = points.get(minIndex).x;
                    	tmpLat = points.get(minIndex).y;
                	} else {
                		// waypoint is near of goal point
                    	tmpLon = goals.get(minIndex2).x;
                    	tmpLat = goals.get(minIndex2).y;
                	}
                }
            	path.add(new Point(tmpLon,tmpLat));
            	int xMode = 0;
            	//if (event.getPressure()>0.35) xMode = 1;
            	if (touchTime>450) xMode = 1;
            	if (touchTime>1050) xMode = 2;
            	wpModes.add(new int[]{xMode});
                lastX = -999;
                lastUpTime = event.getEventTime();
            	buttonTouched = 0;
            	move = 0;
        		//Toast.makeText(getApplicationContext(), ""+touchTime+" / "+path.size(), Toast.LENGTH_SHORT).show();
            	return false;
            }
            if (inputMode==0 && move==0 && buttonTouched==0 && (event.getEventTime()-lastDownTime)>=0 && (event.getEventTime()-lastUpTime)>=0) {
    			touchTime = event.getEventTime() - lastDownTime;
            	if (touchTime>1050 && searchMode>1) {
            		if (Math.abs(azimuth_calib)>1.0) azimuth_calib = 0.0;
            		else azimuth_calib = pathAzimuth - azimuthOK;
                	//if (searchMode==1) return false;
        	    	out[0] = 'n'; // new azimuth
              	    if (!stopped) {
                 	    writeCommand();
                	    tellCommand((char)out[0]);
                		Toast.makeText(getApplicationContext(), "azCalib: "+Math.round(azimuth_calib)+"/"+Math.round(pathAzimuth), Toast.LENGTH_SHORT).show();
                    }
                	return false;
            	}
            }
            lastX = -999;
            lastUpTime = event.getEventTime();
        	buttonTouched = 0;
        	move = 0;
			break;
		case MotionEvent.ACTION_POINTER_DOWN:
        	//tts.speak("pointer down", TextToSpeech.QUEUE_ADD, null);
			if (buttonTouched==0) {
				//move = 0;
				oldDist = spacing(event);
				if (oldDist > 10f) {
					touchMode = 2; // zoom
				}
			}
			break;
		case MotionEvent.ACTION_POINTER_UP:
        	//tts.speak("pointer up", TextToSpeech.QUEUE_ADD, null);
            lastX = -999;
            lastUpTime = event.getEventTime();
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
		            //lastUpTime = event.getEventTime();
				}
			}
			break;
		}
		return true;
    }

    public boolean onTouch(View v, MotionEvent event) {
    	//tts.speak("touch", TextToSpeech.QUEUE_ADD, null);
		//lastDownTime = event.getEventTime();
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        vw = mOpenCvCameraView.getWidth();
        vh = mOpenCvCameraView.getHeight();

        int offset = 0;
        if (rows<300) offset = 0;

        ex = (int)event.getX();
        ey = (int)event.getY();
        x = ex*cols/vw;
        y = ey*rows/vh-offset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");
        
        if ((x>(cols-corner)) && (y<corner)) {
        	// mute corner (top right)
    	    if (voiceOutput>0) voiceOutput = 0; else voiceOutput = 1;
        	if (voiceOutput>0) say("voice");
        	buttonTouched = 1;
    	    return false;
        }
        else if ((x<corner) && (y>(rows-corner)) && searchMode>0) {
        	// debugMode corner (bottom left)
        	if (searchMode>0) {
            	if (inputMode==1) {
            		// remove last waypoint
                	buttonTouched = 2;
            		int lastIndex = path.size() - 1; 
            		if (lastIndex>=0) {
            			path.remove(lastIndex);
            			wpModes.remove(lastIndex);
            		}
                	//if (voiceOutput>0) tts.speak("removed waypoint "+lastIndex, TextToSpeech.QUEUE_ADD, null);
                	//else Toast.makeText(getApplicationContext(), "removed waypoint "+lastIndex, Toast.LENGTH_SHORT).show();
            		return false;
            	}
        	    if (debugMode>0) {
        	    	debugMode = 0;
        	    }
        	    else {
        	    	debugMode = 1;
        	    }
            	if (voiceOutput>0) say("debug "+debugMode);
            	buttonTouched = 2;
				appendLog("QR Droid external call");
				out[0] = 'q'; // adjust center to the left
				saveConfig();
				Intent returnIntent = new Intent();
				returnIntent.putExtra("sMode",""+searchMode);
				returnIntent.putExtra("sWp",""+wp);
				returnIntent.putExtra("sState",state);
				setResult(Activity.RESULT_OK,returnIntent);
				finish();
        	    return false;
			}
        }
        else if ((x<2*corner) && (y>(rows-corner))) {
        	// next corner (bottom left next to debug)
        	if (searchMode>3) {
        		// navigation ON/OFF
        		if (navMode=="on") navMode = "off";
        		else navMode = "on";
            	if (voiceOutput>0) say("nav "+navMode);
            	else Toast.makeText(getApplicationContext(), "nav "+navMode, Toast.LENGTH_SHORT).show();
        	} else if (searchMode>0) {
            	if (wp<(path.size()-1)) computeNextWaypoint(1);
            	else {
            		wp = 0;
        	    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
        	    	goalReached = 0; // we should be at start now
        	    	state = "normal";
            		computeNextWaypoint(1);
            	}
            	if (voiceOutput>0) say("waypoint "+wp+",,, azimuth "+(int)pathAzimuth+",,, distance "+(int)wpDist+",,, mode "+(int)wpMode);
            	else Toast.makeText(getApplicationContext(), "wp "+wp+", az "+(int)pathAzimuth+", dist "+(int)wpDist+", mode "+(int)wpMode, Toast.LENGTH_SHORT).show();
            	buttonTouched = 2;
        	    return false;
        	}
        }
        else if ((x<corner) && (y<corner) && searchMode>0) {
        	// inputMode corner (top left)
    	    if (inputMode>1) {
    	    	inputMode = 0;
    	    	fileName = "RoboNavMap";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	readMap(fileName+".txt");
    	    	fileName = "RoboNavGoals";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	goals = readPoints(fileName+".txt");
    	    	fileName = "RoboNavPath";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	path = readPoints(fileName+".txt");
    	    	//path = readPoints("RoboNavPath.txt");
    	    	wp = 0;
    	    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	    	goalReached = 0; // we should be at start now
    	    	state = "normal";
    	    	computeNextWaypoint(1);
    	    }
    	    else if (inputMode==1) {
    	    	inputMode = 2;
    	    }
    	    else {
    	    	inputMode = 1;
    	    	//path.clear();
    	    	//wpModes.clear();
    	    }
        	if (voiceOutput>0) say("input mode "+inputMode);
        	buttonTouched = 3;
    	    return false;
        }
        else if ((x>(cols-2*corner)) && (y>(rows-corner)) && inputMode<1) {
        	// searchmode corner (bottom right)
        	searchMode++;
    	    if (searchMode>4) searchMode = 0;
    	    if (searchMode>=1) {
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
        	if (voiceOutput>0) say("search mode "+searchMode);
        	buttonTouched = 4;
    	    return false;
        }
        else if ((x<corner) && (y>(rows/2-corner/2)) && (y<(rows/2+corner/2))) {
        	// save area (left center)
        	if (voiceOutput>0) say("saving");
        	saveConfig();
        	// save path
        	String pathText = "";
            for (int i=0; i<path.size(); i++) {
            	pathText += ""+i+" "+path.get(i).y+" "+path.get(i).x+" "+wpModes.get(i)[0]+"\n";
            }
	    	fileName = "RoboNavPath";
	    	if (fileNumber>0) fileName += ""+fileNumber;
        	writeToFile(fileName+".txt",pathText);
        	buttonTouched = 5;
    	    return false;
        }
        else if ((x>=(cols-corner)) && (y<(rows/2)) && (y>=(rows/2-corner-corner2)) && searchMode>0) {
        	// plus rectangle (above center right)
        	buttonTouched = 6;
        	if (voiceOutput>0) say("plus");
        	if (inputMode==1) {
        		// increase fileNumber
                fileNumber += 1;
                if (fileNumber>99) fileNumber = 99;
    	    	//inputMode = 0;
    	    	fileName = "RoboNavMap";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	readMap(fileName+".txt");
    	    	fileName = "RoboNavGoals";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	goals = readPoints(fileName+".txt");
    	    	fileName = "RoboNavPath";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	path = readPoints(fileName+".txt");
    	    	//path = readPoints("RoboNavPath.txt");
    	    	wp = 0;
    	    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	    	goalReached = 0; // we should be at start now
    	    	state = "normal";
    	    	computeNextWaypoint(1);
        		return false;
        	}
        	if (inputMode==2) {
        		// increase startTime
        		startTimeMilis += 5000;
        		setStartTime();
        		return false;
        	}
        	if (searchMode>3) {
                limit1 = limit1 + 1;
                if (limit1>25) limit1 = 25;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mObstacleDetector.setLimits(limit1,limit2);
                shortTxt = "limits: "+limit1+" / "+limit2;
                //mLevel = mLevel + 10;
                //if (mLevel>255) mLevel = 255;
                //mRoadDetector.setLevel(mLevel);
                //mObstacleDetector.setLevel(mLevel);
                //shortTxt = "level: "+mLevel;
        	}
        	else if (searchMode!=2) {
                mLevel = mLevel + 1;
                if (mLevel>255) mLevel = 255;
                mRoadDetector.setLevel(mLevel);
                mObstacleDetector.setLevel(mLevel);
                shortTxt = "level: "+mLevel;
        	}
        	else if (searchMode==2) {
                mArea = mArea + 1;
                if (mArea>255) mArea = 255;
                mColorDetector.setColorRadius(new Scalar(mArea/2,mArea,mArea,mArea));
                shortTxt = "area: "+mArea;
        	}
        	else {
                //minArea = (double)Math.round((minArea + 0.1)*100)/100;
                //if (minArea>1) minArea = 1;
                //mColorDetector.setMinContourArea(minArea);
                limit1 = limit1 + 1;
                if (limit1>25) limit1 = 25;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mObstacleDetector.setLimits(limit1,limit2);
        	}
            if (mColor1==BLUE) mColor1 = YELLOW; else mColor1 = BLUE;
        	mColor1 = RED;
    	    return false;
        }
        else if ((x>=(cols-corner)) && (y>=(rows/2)) && (y<=(rows/2+corner+corner2)) && searchMode>0) {
        	// minus rectangle (below center right)
        	buttonTouched = 7;
        	if (voiceOutput>0) say("minus");
        	if (inputMode==1) {
        		// decrease fileNumber
                fileNumber -= 1;
                if (fileNumber<0) fileNumber = 0;
    	    	//inputMode = 0;
    	    	fileName = "RoboNavMap";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	readMap(fileName+".txt");
    	    	fileName = "RoboNavGoals";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	goals = readPoints(fileName+".txt");
    	    	fileName = "RoboNavPath";
    	    	if (fileNumber>0) fileName += ""+fileNumber;
    	    	path = readPoints(fileName+".txt");
    	    	//path = readPoints("RoboNavPath.txt");
    	    	wp = 0;
    	    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	    	goalReached = 0; // we should be at start now
    	    	state = "normal";
    	    	computeNextWaypoint(1);
        		return false;
        	}
        	if (inputMode==2) {
        		// decrease startTime
        		startTimeMilis -= 5000;
        		setStartTime();
        		return false;
        	}
        	if (searchMode>3) {
                limit1 = limit1 - 1;
                if (limit1<0) limit1 = 0;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mObstacleDetector.setLimits(limit1,limit2);
                shortTxt = "limits: "+limit1+" / "+limit2;
                //mLevel = mLevel - 10;
                //if (mLevel<0) mLevel = 0;
                //mRoadDetector.setLevel(mLevel);
                //mObstacleDetector.setLevel(mLevel);
                //shortTxt = "level: "+mLevel;
        	}
        	else if (searchMode==1 || searchMode==3) {
                mLevel = mLevel - 1;
                if (mLevel<0) mLevel = 0;
                mRoadDetector.setLevel(mLevel);
                mObstacleDetector.setLevel(mLevel);
                shortTxt = "level: "+mLevel;
        	}
        	else if (searchMode==2) {
                mArea = mArea - 1;
                if (mArea<0) mArea = 0;
                mColorDetector.setColorRadius(new Scalar(mArea/2,mArea,mArea,mArea));
                shortTxt = "area: "+mArea;
        	}
        	else {
                //minArea = (double)Math.round((minArea - 0.1)*100)/100;
                //if (minArea<0) minArea = 0;
                //mColorDetector.setMinContourArea(minArea);
                limit1 = limit1 - 1;
                if (limit1<0) limit1 = 0;
                limit2 = limit1 + 10;
                mRoadDetector.setLimits(limit1,limit2);
                mObstacleDetector.setLimits(limit1,limit2);
        	}
            //if (mColor2==BLUE) mColor2 = YELLOW; else mColor2 = BLUE;
        	mColor2 = RED;
    	    return false;
        }
        else if (searchMode==0) {
        	// manual control
        	buttonTouched = 0;
            if (x<=(1.5*w/10)) {
            	// top (left) row
    	    	out[0] = 't'; // turn
            	buttonTouched = 11;
				if (y>(2*h/3)) {
					buttonTouched = 12;
                    appendLog("QR Droid external call");
                    out[0] = 'q'; // adjust center to the left
					Intent returnIntent = new Intent();
					//returnIntent.putExtra("result",result);
					setResult(Activity.RESULT_OK,returnIntent);
					finish();
				}
				else if (y<(1*h/3)) {
					buttonTouched = 13;
                    out[0] = 'e'; // adjust center to the right
				}
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
        
        if (inputMode==1) return false;

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;
        
        if (inputMode<1 && (searchMode==1)) {
        	// targetAzimuth for RR
        	//mTargetAzimuth = (int)azimuthOK;
        	//azimuth_calib = pathAzimuth - azimuthOK;
        	azimuth_calib = pathAzimuth - azimuthOK;
        	//if (searchMode==1) return false;
	    	out[0] = 'n'; // new azimuth
      	    if (!stopped) {
         	    writeCommand();
        	    tellCommand((char)out[0]);
            }
        }
        
        if (inputMode<2) return false;

        Rect touchedRect = new Rect();

        touchedRect.x = (x>1) ? x-1 : 0;
        touchedRect.y = (y>1) ? y-1 : 0;

        touchedRect.width = (x+1 < cols) ? x + 1 - touchedRect.x : cols - touchedRect.x;
        touchedRect.height = (y+1 < rows) ? y + 1 - touchedRect.y : rows - touchedRect.y;

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

        if (searchMode==44 || searchMode==5) {
        	mColorDetector.setHsvColor(mBlobColorHsv);
        	//mColorDetector.setMinArea(mArea);
        	mHSV1 = mBlobColorHsv.toString();
        } else if (searchMode<9) {
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
    	mObstacleDetector.setSearchMode(searchMode);
    	mColorDetector.setSearchMode(searchMode);

    	// searchmode:
    	// 0 = Manual Control (direction, speed)
    	// 1 = RobotemRovne (road search, straight road, azimuth, odometry, crossing roads)
    	// 2 = RoboOrienteering (orange blob search, waypoints, payload drop)
    	// 3 = RoboTour (road search, waypoints, payload drop)
    	// 4 = Road Assistance (not implemented yet)
    	// 5 = Puck Collect (not implemented yet)
    	// 6 = Line Follower (not implemented yet)
    	// 7 = Ketchup House (not implemented yet)
    	// 8 = Bear Rescue (not implemented yet)
    	// 9 = Robo Carts (not implemented yet)
    	
    	if (now.toMillis(false)<=(startTimeMilis+3000)) runMode = 0;
    	else runMode = 1;
    	if (searchMode!=3) runMode = 1; // disabled except RT
        mod2 = mLevel % 2;
        //mod4 = mLevel % 4;
        mod6 = mLevel % 6;

    	if (inputMode<1 && !stopped) {
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
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "l";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(64,64,64);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "r";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(64,64,64);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "s";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 2*w/10;
            	ch = 1*h/6;
        		pta.x = cw-bw/4; pta.y = ch-bh/2;
            	ch = 5*h/6;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "0";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/2;
            	tx = "5";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = 1*h/6;
            	tx = "9";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 4*w/10;
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "f";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = 5*h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "h";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "k";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = 8*w/10;
            	ch = h/2;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "b";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	ch = h/6;
        		pta.x = cw-bw/2; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "p";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
            	cw = (int)(0.5*w)/10;
            	ch = 1*h/6;
        		pta.x = cw-bw/4; pta.y = ch-bh/2;
        		ptb.x = cw+bw/2; ptb.y = ch+bh/2;
        		mColor = new Scalar(128,128,128);
            	Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
            	tx = "e";
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
				cw = (int)(0.5*w)/10;
				ch = 5*h/6;
				pta.x = cw-bw/4; pta.y = ch-bh/2;
				ptb.x = cw+bw/2; ptb.y = ch+bh/2;
				mColor = new Scalar(128,128,128);
				Imgproc.rectangle(mRgba, pta, ptb, mColor, -1);
				tx = "q";
				textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
				Imgproc.putText(mRgba, tx, new Point(cw-2*siz,ch+2*siz), 1, 2*siz, new Scalar(255,255,255), 2*wi);
                //Imgproc.transpose(mRgba, mRgba);
                //w = h;
                //h = mTmp;
            	//if (debugMode>0) Imgproc.putText(mRgba, "o: "+xOffset+" / "+yOffset+" / "+w+" / "+h+" / "+x+" / "+y+" / "+ex+" / "+ey+" / "+vw+" / "+vh, new Point(4,(pos-100)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+btPayload+" t:"+telemetryTable.length()+testTxt+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else if (searchMode==1) {
            	// Road detection (for RobotemRovne) ... direction from moments or topPoint
            	if (blobSearch>=0 && directionNum%2==0) {
                    mRoadDetector.process(mRgba);
                    mContours = mRoadDetector.getContours();
                    //mTemp = mRoadDetector.getResultMat();
                	direction = mRoadDetector.getDirection();
                	centPoint = mRoadDetector.getCentPoint();
                	topPoint = mRoadDetector.getTopPoint();
                	topDirection = mRoadDetector.getTopDirection();
                	topHeight = mRoadDetector.getTopHeight();
                	mLeftOK = mRoadDetector.getLeftOK();
                	mRightOK = mRoadDetector.getRightOK();
                	mCenterOK = mRoadDetector.getCenterOK();
            	}
            	//if (mBoundingRectangle==null) mBoundingRectangle = new Rect(w/2,h/2,1,1);
            	//Imgproc.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Imgproc.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+btPayload+" t:"+telemetryTable.length()+testTxt+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else if (searchMode==2) {
            	// color blob detection (for RoboOrienteering)
            	//blobSearch = 0; // *** for tests only ***
            	//roadSearch = 1; // *** for tests only ***
            	if (blobSearch>=0 && directionNum%3>=0) {
        	    	mColorDetector.setHsvColor(mBlobColorHsv2);
                	mColorDetector.process(mRgba);
                    //mTemp = mColorDetector.getResultMat();
                	blobDirection = mColorDetector.getDirection();
                	mBoundingRectangle = mColorDetector.getBoundingRectangle();
                	// since 2017-05-21
                    mContours = null;
                	direction = 0;
                	centPoint = new Point(w/2,h/w);
                	topPoint = new Point(w/2,0);
                	topDirection = 0;
                	topHeight = 3;
                	mLeftOK = 1;
                	mRightOK = 1;
                	mCenterOK = 1;
               	}
    	    	//Scalar mColorx = new Scalar(0,255,0); // green
    	    	//Scalar mColorx = converScalarHsv2Rgba(mBlobColorHsv2); // blob color
    	    	Scalar mColorx = new Scalar(255,0,255); // violet
    			//if (blobDirection<-limit1 || blobDirection>limit1) mColorx = new Scalar(0,0,255); // blue
    			//if (blobDirection<-limit2 || blobDirection>limit2) mColorx = new Scalar(255,0,0); // red
//    	    	Imgproc.line(mRgba, centerPoint, endPoint, mColorx, 3);
            	if (mBoundingRectangle==null) mBoundingRectangle = new Rect(w/2,h/2,1,1);
            	if (mBoundingRectangle.height>=(h/200)) Imgproc.rectangle(mRgba, mBoundingRectangle.tl(), mBoundingRectangle.br(), mColorx, 4);
            	if (roadSearch>=2 && directionNum%3==1) {
            		// road search disabled since 2017-05-21
                    mRoadDetector.process(mRgba);
                    mContours = mRoadDetector.getContours();
                    //mTemp = mRoadDetector.getResultMat();
                	direction = mRoadDetector.getDirection();
                	centPoint = mRoadDetector.getCentPoint();
                	topPoint = mRoadDetector.getTopPoint();
                	topDirection = mRoadDetector.getTopDirection();
                	topHeight = mRoadDetector.getTopHeight();
                	mLeftOK = mRoadDetector.getLeftOK();
                	mRightOK = mRoadDetector.getRightOK();
                	mCenterOK = mRoadDetector.getCenterOK();
            	}
            	if (roadSearch>=0 && directionNum%3==2) {
                    mObstacleDetector.process(mRgba);
                    //mObstacles = mObstacleDetector.getObstacles();
            	}
            	//Imgproc.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Imgproc.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+btPayload+" t:"+telemetryTable.length()+testTxt+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            } else if (searchMode==3) {
            	// Road detection (for RoboTour) ... direction from topPoint or moments (merging)
            	if (roadSearch>=0 && directionNum%3==0) {
                    mRoadDetector.process(mRgba);
                    mContours = mRoadDetector.getContours();
                    //mTemp = mRoadDetector.getResultMat();
                	direction = mRoadDetector.getDirection();
                	centPoint = mRoadDetector.getCentPoint();
                	topPoint = mRoadDetector.getTopPoint();
                	topDirection = mRoadDetector.getTopDirection();
                	topHeight = mRoadDetector.getTopHeight();
                	mLeftOK = mRoadDetector.getLeftOK();
                	mRightOK = mRoadDetector.getRightOK();
                	mCenterOK = mRoadDetector.getCenterOK();
            	}
            	//if (mBoundingRectangle==null) mBoundingRectangle = new Rect(w/2,h/2,1,1);
            	//Imgproc.putText(mRgba, "c: "+(char)mCommand+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+channel, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "c: "+txtCommand+"/"+tstText+"/"+state+"/"+runMode+" "+searchMode+"/"+mLevel+"/"+limit1+"/"+limit2+"/"+direction+"/"+topDirection+"/"+cameraDirection+"/"+cameraProbability, new Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
            	//Imgproc.putText(mRgba, "s: "+toRgba, new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
            	if (debugMode>0) Imgproc.putText(mRgba, "s: "+azimuth+" "+btDst+" "+btSpd+" "+btRng+" "+btRngLeft+" "+btRngRight+" "+btRngBack+" "+btPwm+" "+btPayload+" t:"+telemetryTable.length()+testTxt+" "+Math.round(100000*btLat)+" "+Math.round(100000*btLon), new Point(4,pos), 1, siz, new Scalar(255,255,50), wi);
    			// merging
            	//direction = topDirection + direction/2; // merging disabled
            } else if (searchMode==4) {
            	// Road Assistance (track good features)
    	    	mColorDetector.setHsvColor(mBlobColorHsv2);
            	mColorDetector.process(mRgba);
                //mTemp = mColorDetector.getResultMat();
            	blobDirection = mColorDetector.getBlobDirection();
            	mBoundingRectangle = mColorDetector.getBoundingRectangle();
            	// since 2017-05-21
                mContours = null;
            	direction = 0;
            	centPoint = new Point(w/2,h/w);
            	topPoint = new Point(w/2,0);
            	topDirection = 0;
            	topHeight = 3;
            	mLeftOK = 1;
            	mRightOK = 1;
            	mCenterOK = 1;
    	    	Scalar mColorx = new Scalar(255,0,255); // violet
            	if (mBoundingRectangle==null) {
            		mBoundingRectangle = new Rect(w/2,h/2,1,1);
            		blobDirection = 0;
            	}
            	if (mBoundingRectangle.height>=(h/200)) Imgproc.rectangle(mRgba, mBoundingRectangle.tl(), mBoundingRectangle.br(), mColorx, 4);
            	trackObject();
            	if (navMode=="on") fastCommand();
            } else if (searchMode==5) {
            	// Puck Collect (track good features and color blobs)
            } else if (searchMode==6) {
            	// Line Follower (track lines and avoid obstacles)
            } else if (searchMode==7) {
            	// Ketchup House (track good features)
            } else if (searchMode==8) {
            	// Bear Rescue (track good features and color blob)
            } else {
            	// Robo Carts (categorize and track good features)
            }
        }
        if (inputMode==2 && searchMode>0) {
       	    // color blob definition (for RoboOrienteering)
	    	mColorDetector.setHsvColor(mBlobColorHsv2);
        	mColorDetector.process(mRgba);
        	blobDirection = mColorDetector.getDirection();
        	mBoundingRectangle = mColorDetector.getBoundingRectangle();
        	if (mBoundingRectangle==null) mBoundingRectangle = new Rect(w/2,h/2,1,1);
        	if (mBoundingRectangle.height>=(h/200)) Imgproc.rectangle(mRgba, mBoundingRectangle.tl(), mBoundingRectangle.br(), VIOLET, 4);
        	mTxt1 = "" + mBlobColorHsv2.toString();
           	textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
        	if (debugMode>=0) Imgproc.putText(mRgba, ""+mTxt1, new Point((w-textSize.width)/2,0.9*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
       	}
        if (inputMode!=1 && searchMode>0) {
            Scalar mBlobColorRgba2 = converScalarHsv2Rgba(mBlobColorHsv2);
        	Mat colorLabel = mRgba.submat(0, corner-1, 0, corner-1);
        	colorLabel.setTo(mBlobColorRgba2);
        }
    	if (inputMode==1) {
        	// inputMode>0 => path defining (black background)
            mRgba.setTo(new Scalar(0,0,0));
        }
    	if (mBoundingRectangle==null) mBoundingRectangle = new Rect(w/2,h/2,1,1);
    	if (searchMode==0) {
        	btConnected = MainActivity.mSerialService.getState();
        	mTxt1 = "BT";
           	//textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
        	if (debugMode>=0 && btConnected>2) Imgproc.putText(mRgba, mTxt1, new Point(corner/2-textSize.width,h-corner/2), 1, siz*1.5, new Scalar(0,255,0), 3*wi/2);
        	else Imgproc.putText(mRgba, mTxt1, new Point(corner/2-textSize.width,h-corner/2), 1, siz*1.5, new Scalar(255,0,0), 3*wi/2);
    	}
        if (searchMode!=0 && !stopped) {
//        	//Imgproc.line(mRgba, new Point(mBoundingRectangle.x, mBoundingRectangle.y), new Point(w/2,h), new Scalar(255,255,50), 2);
//        	if (mBoundingRectangle.height>9) Imgproc.rectangle(mRgba, mBoundingRectangle.br(), mBoundingRectangle.tl(), new Scalar(255,255,50), 2);
//        	// add main information to mRgba
        	double tmpx = 0;
        	double tmpy = 0;
        	int[] edge = {0,0,0,0};
        	if (inputMode<1) {
        		//mRgba = mTemp;
                //Vector<Mat> channels = new Vector<Mat>();
                //Core.split(mTemp, channels);
                //channels.set(0,mTemp);
                //channels.set(1,mTemp);
                //channels.set(2,mTemp);
                //channels.set(0,new Mat(h, w, 0, new Scalar(0)));
                //channels.set(0,mTemp);
                //channels.set(1,mTemp);
                //channels.set(2,mTemp);
            	//Core.add(mRgba, mTemp, mRgba);
                //Core.add(mRgba, new Scalar(100), mRgba);
                //Core.add(mRgba, channels.get(0), mRgba);
                //Core.merge(channels, mRgba);
				if (mContours!=null && mRgba!=null && mContours.size()>0) {
					try {
						Imgproc.drawContours(mRgba, mContours, 0, new Scalar(255,0,0), 4);
					} catch (Exception e) {
						e.printStackTrace();
					}
				}
            	//if (mRgba!=null && mContours!=null && mContours.size()>0) Imgproc.drawContours(mRgba, mContours, 0, new Scalar(255,0,0), 4);
            	//if (debugMode>=0) Core.putText(mRgba, ""+mLevel+" "+cameraDirection+" "+" "+Math.round(drivingSignal)+" "+mod4, new Point(w/6,h-h/6), 1, siz*2, new Scalar(255,255,50), wi*3);
            	mTxt1 = ""+String.format("%4d", (int)azimuthOK);
            	mTxt2 = ""+String.format("%4d", (int)mTargetAzimuth);
            	mTxt3 = ""+String.format("%4d", (int)compassAzimuth);
            	//mTxt3 = ""+String.format("%4d/%4d/%4d", (int)azimDiffOK, (int)azimDiff, (int)turnAngleToNextWaypoint);
            	mTxt4 = ""+String.format("%4d/%4d/%4d", (int)drivingSignal, (int)mBoundingRectangle.height, (int)blobDirection);
            	//mTxt5 = " "+(char)(out1[0] & 0xFF)+"/"+mLeftOK+"/"+mRightOK;
            	mTxt5 = ""+String.format("%4d", (int)commandsMap.size());
            	//mTxt5 = ""+String.format("%4d", (int)azimLimit);
            	//mTxt5 = ""+String.format("%4d", (int)pathAzimuth);
            	//mTxt6 = ""+String.format("%4d", (int)azimuthToNextWaypoint);
            	mTxt6 = ""+String.format("%4d", (int)goals.size());
            	//mTxt6 = ""+String.format("%4d", (int)blobDirection);
            	mTxt7 = ""+String.format("%4d", (int)mod6);
            	//mTxt5 = "a:"+String.format("%4d", (int)azimuthOK);
            	if (searchMode==2) {
                	//mTxt2 = ""+state;
                	//mTxt3 = ""+String.format("%4d", (int)blobDirection);
                	//mTxt4 = ""+String.format("%4d", (int)mBoundingRectangle.height);
            	}
            	mTxt1 = mTxt1+" "+mTxt2+" "+" "+mTxt3+" "+mTxt4+" "+mTxt5+" "+mTxt6+" "+mTxt7;
            	//mTxt1 = "a:"+(int)azimuthOK+"/"+mObstacles.size();
            	//mTxt1 += " p:"+(int)pathAzimuth;
            	//mTxt1 += " w:"+(int)azimuthToNextWaypoint;
            	//mTxt1 += " c:"+(int)compassAzimuth;
            	//mTxt1 += " d:"+(int)drivingSignal;
            	//mTxt1 += " "+(int)turnAngleToNextWaypoint+"/"+(int)turnAngleToStayOnRoad+"/"+(int)turnAngleToAvoidObstacle;
            	//mTxt1 += " "+mLeftOK+"/"+mCenterOK+"/"+mRightOK;
            	//mTxt1 = ""+mBlobColorHsv2.toString();
            	//mTxt1 += " "+btRngLeftL+" "+btRngLeft+" "+btRngLeftR+" "+btRngL+" "+btRng+" "+btRngR+" "+btRngRightL+" "+btRngRight+" "+btRngRightR;
            	mTxt1 = ""+(int)azimuthOK+"/"+(int)pathAzimuth+"/"+(int)azimuthToNextWaypoint+"/"+(int)azimDiff+"/"+(int)blobDirection;;
            	//if (searchMode==2) mTxt1 = "" + mBlobColorHsv2.toString() + "/" + (int)mArea + " # " + mTxt1;
            	if (searchMode==2) mTxt1 = ""+(int)mArea+"#"+(int)azimuthToNextWaypoint+"/"+(int)azimDiff+"#"+(int)blobDirection+"/"+(int)mBoundingRectangle.y+"#"+wpMode+"/"+(int)distanceToNextWaypoint;
            	else if (searchMode==4) mTxt1 = ""+(int)blobDirection+" x:"+(int)mBoundingRectangle.x+" y:"+(int)mBoundingRectangle.y;
               	textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
            	//if (debugMode>=0) Imgproc.putText(mRgba, ""+mTxt1+" "+mTxt2+" "+" "+mTxt3+" "+mTxt4+" "+mTxt5+" "+mTxt6+" "+mTxt7, new Point(1,0.7*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
            	if (searchMode==4) Imgproc.putText(mRgba, ""+mTxt1, new Point((w-textSize.width)/2,0.9*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
				else if (searchMode==2) Imgproc.putText(mRgba, ""+mTxt4, new Point((w-textSize.width)/2,0.9*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
            	if (debugMode>0) {
            		//Imgproc.putText(mRgba, ""+mTxt1, new Point((w-textSize.width)/2,0.9*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
            	}
            	else if (shortTxt!="") {
                	long time = System.currentTimeMillis();
            		if (shortNum==0) lastShow = time + 300;
            		shortNum++;
                   	textSize = Imgproc.getTextSize(shortTxt, 1, 1.4*siz, 14*wi/10, baseline);
                	Imgproc.putText(mRgba, ""+shortTxt, new Point((w-textSize.width)/2,0.8*h), 1, siz*1.4, new Scalar(255,255,50), 14*wi/10);
                	if (shortNum>20 || lastShow<time) {
                		shortTxt = "";
                		shortNum = 0;
                	}
            	}
            	mTxt1 = ""+state+" "+mText+" "+txtCommand;
            	mTxt1 = ""+txtCommand;
               	textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
            	if (debugMode>=0) Imgproc.putText(mRgba, mTxt1, new Point((w-textSize.width)/2,h-h/60), 1, siz*1.5, new Scalar(255,0,255), 3*wi/2);
            	mTxt1 = ""+mText;
               	textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
            	if (debugMode>=0) Imgproc.putText(mRgba, mTxt1, new Point(w/5,h-h/60), 1, siz*1.5, new Scalar(255,0,255), 3*wi/2);
            	btConnected = MainActivity.mSerialService.getState();
            	mTxt1 = "BT";
               	//textSize = Imgproc.getTextSize(mTxt1, 1, 1.4*siz, 14*wi/10, baseline);
            	if (debugMode>=0 && btConnected>2) Imgproc.putText(mRgba, mTxt1, new Point(2*w/3,h-h/60), 1, siz*1.5, new Scalar(0,255,0), 3*wi/2);
            	else Imgproc.putText(mRgba, mTxt1, new Point(2*w/3,h-h/60), 1, siz*1.5, new Scalar(255,0,0), 3*wi/2);
    			Imgproc.circle(mRgba, topPoint, h/50, new Scalar(255,0,0), -1);
    			Imgproc.circle(mRgba, centPoint, h/50, new Scalar(0,0,255), -1);
            	int ok = 0;
            	Scalar tmpColor = new Scalar(200,200,200);
    	    	Scalar mColorx = new Scalar(0,255,0); // green
    			if (drivingSignal<-limit1 || drivingSignal>limit1) mColorx = new Scalar(0,0,255); // blue
    			if (drivingSignal<-limit2 || drivingSignal>limit2) mColorx = new Scalar(255,0,0); // red
            	// show command graphically
            	if (mOrientation==2) {
        			// landscape 
            		tmpy = h*(0.75-0.6*Math.cos(drivingSignal/36));
            		tmpx = 0.5*w+0.6*h*Math.sin(drivingSignal/36);
        			Imgproc.line(mRgba, new Point(0.5*w,0.75*h), new Point(tmpx,tmpy), mColorx, 8);
        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,0,0), -1);
//            		tmpy = h*(0.75-0.5*Math.cos(turnAngleToStayOnRoad/36));
//            		tmpx = 0.5*w+0.5*h*Math.sin(turnAngleToStayOnRoad/36);
//        			Imgproc.line(mRgba, new Point(0.5*w,0.75*h), new Point(tmpx,tmpy), new Scalar(128,255,255), 4);
//        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(128,255,255), -1);
//            		tmpy = h*(0.75-0.4*Math.cos(turnAngleToNextWaypoint/36));
//            		tmpx = 0.5*w+0.4*h*Math.sin(turnAngleToNextWaypoint/36);
//        			Imgproc.line(mRgba, new Point(0.5*w,0.75*h), new Point(tmpx,tmpy), new Scalar(255,128,255), 4);
//        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,128,255), -1);
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
        			Imgproc.line(mRgba, new Point(0.75*w,0.5*h), new Point(tmpx,tmpy), mColorx, 5);
        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,0,0), -1);
//            		tmpx = h*(0.75-0.5*Math.cos(turnAngleToStayOnRoad/36));
//            		tmpy = 0.5*w+0.5*h*Math.sin(turnAngleToStayOnRoad/36);
//        			Imgproc.line(mRgba, new Point(0.75*w,0.5*h), new Point(tmpx,tmpy), new Scalar(128,255,255), 4);
//        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(128,255,255), -1);
//            		tmpx = h*(0.75-0.4*Math.cos(turnAngleToNextWaypoint/36));
//            		tmpy = 0.5*w+0.4*h*Math.sin(turnAngleToNextWaypoint/36);
//        			Imgproc.line(mRgba, new Point(0.75*w,0.5*h), new Point(tmpx,tmpy), new Scalar(255,128,255), 4);
//        			Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/100, new Scalar(255,128,255), -1);
        			//Imgproc.line(mRgba, new Point(topPoint.x,topPoint.y), new Point(w-2*lin,h/2), new Scalar(0), 1);
                	if (txtCommand=="xleft") {ok = 1; tmpx = w/2; tmpy = 3*h/4; tmpColor = new Scalar(255,0,0);}
                	else if (txtCommand=="xright") {ok = 1; tmpx = w/2; tmpy = h/4; tmpColor = new Scalar(255,0,0);}
                	else if (txtCommand=="forward") {ok = 1; tmpx = w/4; tmpy = h/2; tmpColor = new Scalar(0,255,0);}
                	else if (txtCommand=="straigh") {ok = 1; tmpx = w/4; tmpy = h/2; tmpColor = new Scalar(255,255,255);}
                	else if (txtCommand=="left") {ok = 1; tmpx = w/3; tmpy = 2*h/3; tmpColor = new Scalar(0,0,255);}
                	else if (txtCommand=="right") {ok = 1; tmpx = w/3; tmpy = h/3; tmpColor = new Scalar(0,0,255);}
                	else if (txtCommand=="stop") {ok = 1; tmpx = w/2; tmpy = h/2; tmpColor = new Scalar(0,0,0);}
                	else if (txtCommand=="back") {ok = 1; tmpx = 0.9*w; tmpy = h/2; tmpColor = new Scalar(255,0,0);}
        		}
        		if (ok>0) Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/20, tmpColor, -1);
        		if (azimuthOK!=999 && searchMode!=4) {
        			// add aimuthOK to mRgba
            		tmpx = w*0.5+h*0.2*Math.sin(Math.toRadians(azimuthOK));
            		tmpy = h*(0.5-0.2*Math.cos(Math.toRadians(azimuthOK)));
        			Imgproc.line(mRgba, new Point(0.5*w,0.5*h), new Point(tmpx,tmpy), RED, 6);
        			//Imgproc.circle(mRgba, new Point(tmpx,tmpy), h/50, RED, -1);
        		}
            	getLocation();
            	if (inputMode<1) if (debugMode>0) Imgproc.putText(mRgba, "a: "+azimuth+"/"+azimuthValid+"/"+azimuthLimit+"/"+mArea+"/"+minArea, new Point(4,(h/4)), 1, siz, new Scalar(255,255,150), wi);
            	if (inputMode<1) if (debugMode>0) {
        		  	SimpleDateFormat format = new SimpleDateFormat("HHmmss",Locale.US);
        		  	String GPStime = format.format(new Date(lastGPStime));
        		  	String NETtime = format.format(new Date(lastNETtime));
            		Imgproc.putText(mRgba, "g: "+stav+"/"+Math.round(accuracy)+"/"+Math.round(bearing)+"/"+Math.round(100000*lat)+"/"+Math.round(100000*lon)+"/"+GPStime+"/"+NETtime, new Point(4,(3*h/8)), 1, siz, new Scalar(255,255,150), wi);
            	}
            	computePosition(); // compute actual position (estimation, latOK, lonOK, azimuthOK, distanceOK)
            	if (edges.size()>0)  edge = edges.get(0);
            	if (inputMode<1) {
            		if (debugMode>0) Imgproc.putText(mRgba, "p: "+path.size()+"/"+wp+"/"+wpMode+"/"+Math.round(wpDist)+"/"+Math.round(pathAzimuth)+"/"+Math.round(1000*wpLat)/1000+"/"+Math.round(1000*wpLon)/1000, new Point(corner,(h/2)), 1, siz, new Scalar(255,255,150), wi);
                	if (goals.size()>1) if (debugMode>0) Imgproc.putText(mRgba, "t: "+goals.size()+"/"+Math.round(1000*goals.get(0).x)/1000+"/"+Math.round(1000*goals.get(0).y)/1000+"/"+Math.round(1000*goals.get(1).x)/1000+"/"+Math.round(1000*goals.get(1).y)/1000+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
                	else if (goals.size()>0) if (debugMode>0) Imgproc.putText(mRgba, "t: "+goals.size()+"/"+Math.round(1000*goals.get(0).x)/1000+"/"+Math.round(1000*goals.get(0).y)/1000+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
                	else if (debugMode>0) Imgproc.putText(mRgba, "t: "+goals.size()+" "+edges.size()+"/"+edge[0]+"/"+edge[1]+"/"+edge[2]+"/"+edge[3], new Point(4,(5*h/8)+10), 1, siz, new Scalar(255,255,150), wi);
            	}
        	}
        	double tmpx0 = 0;
        	double tmpy0 = 0;
        	tmpx = 0;
        	tmpy = 0;
        	if (inputMode<2 && searchMode<=3) {
        		try {
					// add map points to mRgba
					if (points!=null && points.size()>0) {
						for (int i=0; i<points.size(); i++) {
							tmpx = (points.get(i).x - centLon)*multLon+w/2;
							tmpy = (centLat - points.get(i).y)*multLat+h/2;
							Imgproc.circle(mRgba, new Point(tmpx,tmpy), 4, new Scalar(0,0,255),-1);
						}
					}
					// add edges to mRgba
					if (edges!=null && edges.size()>0) {
						for (int i=0; i<edges.size(); i++) {
							edge = edges.get(i);
							int p1 = edge[0];
							int p2 = edge[1];
							if (p1<points.size() && p2<points.size()) {
								tmpx0 = (points.get(p1).x - centLon)*multLon+w/2;
								tmpy0 = (centLat - points.get(p1).y)*multLat+h/2;
								tmpx = (points.get(p2).x - centLon)*multLon+w/2;
								tmpy = (centLat - points.get(p2).y)*multLat+h/2;
								Imgproc.line(mRgba, new Point(tmpx0,tmpy0), new Point(tmpx,tmpy), new Scalar(0,0,255),2);
							}
						}
					}
					// add goals to mRgba
					if (goals!=null) {
						for (int i=0; i<goals.size(); i++) {
							tmpx = (goals.get(i).x - centLon)*multLon+w/2;
							tmpy = (centLat - goals.get(i).y)*multLat+h/2;
							Imgproc.circle(mRgba, new Point(tmpx+3,tmpy+0), 4, new Scalar(255,0,0),-1);
							if (i<goalPoints.size()) {
								Imgproc.putText(mRgba, ""+i+"/"+goalPoints.get(i)[0], new Point(tmpx,tmpy), 1, siz, new Scalar(255,255,255), wi);
							}
						}
					}
					// add path to mRgba
					if (path!=null) {
						for (int i=0; i<path.size(); i++) {
							tmpx = (path.get(i).x - centLon)*multLon+w/2+1;
							tmpy = (centLat - path.get(i).y)*multLat+h/2+1;
							if (i==wp) Imgproc.circle(mRgba, new Point(tmpx-3,tmpy-0), 10, new Scalar(200,200,200),-1);
							if (i<wpModes.size()) {
								if (wpModes.get(i)[0]==1) Imgproc.circle(mRgba, new Point(tmpx-3,tmpy-0), 5, new Scalar(255,153,0),-1);
								else if (wpModes.get(i)[0]==2) Imgproc.circle(mRgba, new Point(tmpx-3,tmpy-0), 6, new Scalar(255,0,0),-1);
								else Imgproc.circle(mRgba, new Point(tmpx-3,tmpy-0), 4, new Scalar(0,255,0),-1);
							}
							if (i>0) {
								if (i==wp) Imgproc.line(mRgba, new Point(tmpx0,tmpy0), new Point(tmpx,tmpy), new Scalar(255,165,0),4);
								else Imgproc.line(mRgba, new Point(tmpx0,tmpy0), new Point(tmpx,tmpy), new Scalar(0,255,0),1);
							}
							tmpx0 = tmpx;
							tmpy0 = tmpy;
						}
					}
					// add actual GPS position to mRgba
					if (latOK>30f) {
						tmpx = (lonOK - centLon)*multLon+w/2;
						tmpy = (centLat - latOK)*multLat+h/2;
						Imgproc.circle(mRgba, new Point(tmpx,tmpy), 6, new Scalar(255,0,0),-1);
					}
				} catch (Exception e) {
					//e.printStackTrace();
				}
        	}
        	mColor = RED;
        	tx = ""+fileNumber;
        	if (fileNumber<maps.length) tx = maps[fileNumber]; 
        	if (inputMode==1) {mColor = GREEN;}
        	else if (inputMode>1) {mColor = BLUE;}
        	if (searchMode!=0) {
        		Imgproc.rectangle(mRgba, new Point(1,1), new Point(corner-1,corner-1), converScalarHsv2Rgba(mBlobColorHsv2), -1);
        	}
        	Imgproc.rectangle(mRgba, new Point(0,0), new Point(corner,corner), mColor, 2);
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
        	mColor1 = BLUE;
        	if (mod6==1 || mod6==3 || mod6==5) mColor1 = GREEN;
        	mColor2 = BLUE;
        	if (mod6>=2) mColor2 = GREEN;
        	if (mod6>=4) mColor2 = BLACK;
        	Imgproc.rectangle(mRgba, pt5, pt6, mColor1, -1);
        	Imgproc.rectangle(mRgba, pt7, pt8, mColor2, -1);
        	if (mod2>0) {
        		//mColor1 = BLUE;
        		azimLimit = 30;
        	} else {
        		//mColor1 = BLACK;
        		azimLimit = 20;
        	}
        	tx = "+";
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,h/2-corner2-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,0,0), 2*wi);
        	tx = "-";
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,h/2+corner2+corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,0,0), 2*wi);
        	now.setToNow();
        	tx = now.format("%H:%M:%S");
        	tx += " ("+startTime+")";
        	Imgproc.putText(mRgba, tx, new Point(w/4,h/25+2), 1, siz, new Scalar(255,255,100), wi);
        }
        if (!stopped) {
        	if (searchMode>0) {
            	mColor = RED; tx = "D";
            	if (debugMode>0) {mColor = GREEN; tx = "D";}
            	Imgproc.rectangle(mRgba, new Point(0,h), new Point(corner,h-corner), mColor, 1);
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,h-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
            	mColor = RED; tx = "N";
            	if (navMode=="on") mColor = GREEN;
            	if (searchMode>3) tx += navMode;
            	Imgproc.rectangle(mRgba, new Point(corner,h), new Point(2*corner,h-corner), mColor, 1);
            	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
            	Imgproc.putText(mRgba, tx, new Point(3*corner/2-textSize.width/2+1,h-corner/2+textSize.height/2+1), 1, 2*siz, mColor, 2*wi);
        	}
        	mColor = RED; tx = "M";
        	if (voiceOutput>0) {mColor = GREEN; tx = "V";}
        	Imgproc.rectangle(mRgba, pt1, pt2, mColor, -1);
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(w-corner/2-textSize.width/2+1,corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(0,0,255), 2*wi);
        	mColor = WHITE;
        	if (searchMode>0) mColor = BLACK;
        	if (searchMode==1) mColor = BLUE;
        	else if (searchMode==2) mColor = YELLOW;
        	else if (searchMode==3) mColor = GREEN;
        	else if (searchMode==4) mColor = RED;
        	if (inputMode<1) Imgproc.rectangle(mRgba, pt3, pt4, mColor, -1);
        	tx = ""+searchMode+sm[searchMode];
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(w-corner-textSize.width/2+1,h-corner/2+textSize.height/2+1), 1, 2*siz, new Scalar(200,200,200), 2*wi);
        	Imgproc.rectangle(mRgba, new Point(0,h/2+corner/2), new Point(corner,h/2-corner/2), BLACK, -1);
        	tx = "S";
        	textSize = Imgproc.getTextSize(tx, 1, 2*siz, 2*wi, baseline);
        	Imgproc.putText(mRgba, tx, new Point(corner/2-textSize.width/2+1,h/2+textSize.height/2+1), 1, 2*siz, new Scalar(255,255,255), 2*wi);
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
//            	if (toRgba.length()>=telemetryTable.length()) {
              	if (toRgba.length()>=3) {
            		btValid = 1;
            		int tmp = 0;
            		tmp = Integer.parseInt(toRgba.substring(fromHeading,toHeading).trim());
                	if (tmp>0) {
                		azimuth = tmp;
                		now.setToNow();
                		azimuthValidTime = now.toMillis(false) + 3000;
                		azimuthValid = 1;
                    	//azimuthDifference = azimuth - initialAzimuth;
                    	azimuthDifference = (int)(azimuth - azimuthToNextWaypoint);
                    	if (azimuthDifference<-180) azimuthDifference += 360;
                    	else if (azimuthDifference>180) azimuthDifference -= 360;
                    	if (toPwm>0 && toPwm<=toRgba.length()) btPwm = Integer.parseInt(toRgba.substring(fromPwm,toPwm).trim());
                    	if (toFront>0 && toFront<=toRgba.length()) btRng = Integer.parseInt(toRgba.substring(fromFront,toFront).trim());
                    	if (btRng>100) {
                        	//btRngL = 4*Integer.parseInt(toRgba.substring(24,24).trim());
                        	//btRng = 4*Integer.parseInt(toRgba.substring(25,25).trim());
                        	//btRngR = 4*Integer.parseInt(toRgba.substring(26,26).trim());
                        	btRngL = 4*(int)Math.floor(btRng/100);
                        	btRngR = 4*((int)btRng % 10);
                        	btRng = 4*(int)Math.floor((btRng % 100)/10);
                    	}
                    	if (toDistance>0 && toDistance<=toRgba.length()) btDst = Integer.parseInt(toRgba.substring(fromDistance,toDistance).trim());
                    	if (toSpeed>0 && toSpeed<=toRgba.length()) btSpd = Integer.parseInt(toRgba.substring(fromSpeed,toSpeed).trim());
                		if (toLat>0 && toLat<=toRgba.length()) btLat = Float.parseFloat(toRgba.substring(fromLat,toLat).trim());
                		if (toLon>0 && toLon<=toRgba.length()) btLon = Float.parseFloat(toRgba.substring(fromLon,toLon).trim());
                		if (toLeft>0 && toLeft<=toRgba.length()) btRngLeft = Integer.parseInt(toRgba.substring(fromLeft,toLeft).trim());
                    	if (btRngLeft>100) {
                        	//btRngLeftL = 4*Integer.parseInt(toRgba.substring(58,58).trim());
                        	//btRngLeft = 4*Integer.parseInt(toRgba.substring(59,59).trim());
                        	//btRngLeftR = 4*Integer.parseInt(toRgba.substring(60,60).trim());
                        	btRngLeftL = 4*(int)Math.floor(btRngLeft/100);
                        	btRngLeftR = 4*((int)btRngLeft % 10);
                        	btRngLeft = 4*(int)Math.floor((btRngLeft % 100)/10);
                    	} else {
                        	btRngLeftL = btRngLeft;
                        	btRngLeftR = btRngLeft;
                    	}
                    	if (toRight>0 && toRight<=toRgba.length()) btRngRight = Integer.parseInt(toRgba.substring(fromRight,toRight).trim());
                    	if (btRngRight>100) {
                        	//btRngRightL = 4*Integer.parseInt(toRgba.substring(62,62).trim());
                        	//btRngRight = 4*Integer.parseInt(toRgba.substring(63,63).trim());
                        	//btRngRightR = 4*Integer.parseInt(toRgba.substring(64,64).trim());
                        	btRngRightL = 4*(int)Math.floor(btRngRight/100);
                        	btRngRightR = 4*((int)btRngRight % 10);
                        	btRngRight = 4*(int)Math.floor((btRngRight % 100)/10);
                    	} else {
                        	btRngRightL = btRngRight;
                        	btRngRightR = btRngRight;
                    	}
                    	if (toBack>0 && toBack<=toRgba.length()) btRngBack = Integer.parseInt(toRgba.substring(fromBack,toBack).trim());
                		if (toPwm>0 && toPwm<=toRgba.length()) btPwm = Integer.parseInt(toRgba.substring(fromPwm,toPwm).trim());
                    	if (toPayload>0 && toPayload<=toRgba.length()) btPayload = Integer.parseInt(toRgba.substring(fromPayload,toPayload).trim());
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

	public static void setState(int sMode, int sWp, String sState, String sQrCode) {
		searchMode = sMode;
		wp = sWp;
		state = sState;
		qrCode = sQrCode;
		//computeNextWaypoint(0);
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
    
    static void setConnectedAddress(String sAddress) {
    	connectedAddress = sAddress;
		//testTxt += "a";
        //Toast.makeText(getApplicationContext(), "line: "+line, Toast.LENGTH_SHORT).show();
    	robots = readRobots("RoboNavRobots.txt");
//		for (int x=0; x<robots.size(); x++) {
//	    	String[] parts = robots.get(x).split(";");
//	    	if (connectedAddress.equalsIgnoreCase(parts[0].trim())) {
//	    		testTxt += "f";
//	    		robotName = parts[1];
//	            //Toast.makeText(getApplicationContext(), "obot: "+robotName, Toast.LENGTH_SHORT).show();
//	    		if (parts[2].length()>=commandsTable0.length()) {
//		    		commandsTable = parts[2];
//		    		commandsMap.clear();
//		    		for (int ix=0; ix<commandsTable0.length(); ix++) {
//		    			commandsMap.put(commandsTable0.charAt(ix), commandsTable.charAt(ix));
//		    		}
//	    		}
//	    		if (parts[3].length()>0) {
//		    		telemetryTable = ""+parts[3];
//		    		testTxt += ":"+telemetryTable.length();
//		    		parseTelemetryTable();
//	    		}
//	    		break;
//	    	}
//		}
    }

    static String getAllowedAddresses() {
    	robots = readRobots("RoboNavRobots.txt");
    	return allowedAddresses;
    }

    static String getRobotName() {
    	return robotName;
    }

    static void parseTelemetryTable() {
		//testTxt += "p";
    	fromHeading = telemetryTable.indexOf("h");
    	toHeading = telemetryTable.lastIndexOf("h")+1;
    	fromSpeed = telemetryTable.indexOf("s");
    	toSpeed = telemetryTable.lastIndexOf("s")+1;
    	fromDistance = telemetryTable.indexOf("d");
    	toDistance = telemetryTable.lastIndexOf("d")+1;
    	fromPwm = telemetryTable.indexOf("w");
    	toPwm = telemetryTable.lastIndexOf("w")+1;
    	fromLat = telemetryTable.indexOf("a");
    	toLat = telemetryTable.lastIndexOf("a")+1;
    	fromLon = telemetryTable.indexOf("o");
    	toLon = telemetryTable.lastIndexOf("o")+1;
    	fromFront = telemetryTable.indexOf("f");
    	toFront = telemetryTable.lastIndexOf("f")+1;
    	fromLeft = telemetryTable.indexOf("l");
    	toLeft = telemetryTable.lastIndexOf("l")+1;
    	fromRight = telemetryTable.indexOf("r");
    	toRight = telemetryTable.lastIndexOf("r")+1;
    	fromBack = telemetryTable.indexOf("b");
    	toBack = telemetryTable.lastIndexOf("b")+1;
    	fromPayload = telemetryTable.indexOf("p");
    	toPayload = telemetryTable.lastIndexOf("p")+1;
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
            mText = "";
          	if (!stopped && searchMode>0 && inputMode<1) {
               	txtCommand = "";
                //if (stav=="OK" && lat>30f && inputMode<1 && wpDist1<3000) {
                if (latOK>30f && inputMode<1 && wpDist1<3000) {
                	centLat = latOK;
                	centLon = lonOK;
                    //Toast.makeText(getApplicationContext(), "center", Toast.LENGTH_SHORT).show();
                }
          		// countdown
          		now.setToNow();
          		long tmpTime = now.toMillis(false);
      			long runTime = (tmpTime - firstTime)/1000;
          		if (searchMode==1 && runTime<10) {
                	//azimuth_calib += pathAzimuth - azimuthOK;
          		}
      			tmpSec = (startTimeMilis - tmpTime)/1000;
          		if (runMode<1) {
          			// countdown active
          	    	String txtCommand1 = "";
          	    	if (tmpSec==120) txtCommand1 = "2 minutes";
          	    	else if (tmpSec==60) txtCommand1 = "1 minute";
          	    	else if (tmpSec==50) {
          	    		if (latOK>30f) txtCommand1 = "GPS ready";
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
          		    	say(txtCommand1);
      		    	}
              		computeCommand(); // main navigation algorithm => mCommand
              		mCommand = '-';
        	    	out[0] = mCommand;
                 	txtCommand = "";
      		    	return;
          		}
          		computeCommand(); // main navigation algorithm => mCommand
          		if (searchMode==3 && tmpSec>=-5) mCommand = 'w'; // initial mCommand for RoboTour is "forward"
             	txtCommand = "";
          		if (searchMode<=3) {
          			// send "slow" command (once per second) for defined modes only
          			slowCommand();
          		}
            	SimpleDateFormat format = new SimpleDateFormat("yyMMdd_HHmmss",Locale.US);
            	String dateTimeString = format.format(new Date());
                //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+direction+";"+topDirection+";"+averageDirection+";"+averageTopPoint+";"+directionNum+";"+azimuth+";"+initialAzimuth+";"+azimuthDifference+";"+azimuthValid+";"+btPwm+";"+btRng+";"+btDst+";"+btSpd+";"+limit1+";"+limit2+";"+mLevel+";"+azimuthLimit+";"+mArea+";"+minArea+";"+btLat+";"+btLon+";"+btRngLeft+";"+btRngRight+";"+btRngBack+";");
            	// new log structure (4.9.2015)
                //appendLog("date_time;mCommand;searchMode;direction;topDirection;cameraDirection;directionNum;azimuth;azimuthValid;btPwm;btRng;btRngLeft;btRngRight;btRngBack;btDst;btSpd;limit1;limit2;mLevel;azimuthLimit;mArea;minArea;btLat;btLon;stav;lat;lon;bearing;accuracy;wp;wpLat;wpLon;wpMode;wpDist;pathAzimuth;wpDist1;azimuthToNextWaypoint;azimuth;actualDist;lastDist;");
                //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+direction+";"+topDirection+";"+cameraDirection+";"+azimuth+";"+azimuthValid+";"+btPwm+";"+btRng+";"+btRngLeft+";"+btRngRight+";"+btRngBack+";"+btDst+";"+btSpd+";"+limit1+";"+limit2+";"+mLevel+";"+azimuthLimit+";"+mArea+";"+minArea+";"+Math.round(100000*btLat)+";"+Math.round(100000*btLon)+";"+stav+";"+Math.round(100000*lat)+";"+Math.round(100000*lon)+";"+Math.round(bearing)+";"+Math.round(accuracy)+";"+wp+";"+Math.round(100000*wpLat)+";"+Math.round(100000*wpLon)+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(pathAzimuth)+";"+Math.round(wpDist1)+";"+Math.round(azimuthToNextWaypoint)+";"+azimuth+";"+Math.round(azimDiff)+";"+Math.round(turnAngleToNextWaypoint)+";"+Math.round(actualDist)+";"+Math.round(lastDist)+";");
            	// simplified log structure (26.11.2017)
                //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+azimuthValid+";"+Math.round(100000*lat)+";"+Math.round(100000*lon)+";"+Math.round(bearing)+";"+wp+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(pathAzimuth)+";"+Math.round(azimuthToNextWaypoint)+";");
            	// another log structure (27.11.2017)
                //appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+azimuthValid+";"+btDst+";"+btSpd+";"+Math.round(100000*lat)+";"+Math.round(100000*lon)+";"+Math.round(bearing)+";"+wp+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(pathAzimuth)+";"+Math.round(azimuthToNextWaypoint)+";");
            	// another log structure (5.12.2017)
                appendLog(dateTimeString+";"+(char)out[0]+";"+searchMode+";"+azimuthOK+";"+btDst+";"+btSpd+";"+Math.round(100000*latOK)+";"+Math.round(100000*lonOK)+";"+Math.round(bearing)+";"+wp+";"+wpMode+";"+Math.round(wpDist)+";"+Math.round(pathAzimuth)+";"+Math.round(azimuthToNextWaypoint)+";"+Math.round(drivingSignal)+";");
                //directionTrend = -direction/2;
                //topPointTrend = -topDirection/2;
             	if (mPrevCommand=='l' || mPrevCommand=='r') {
                    directionTrend = -directionTrend;
                    topPointTrend = -topPointTrend;
             	} else if (mPrevCommand=='h' || mPrevCommand=='k') {
                        directionTrend = -directionTrend/2;
                        topPointTrend = -topPointTrend/2;
                } else {
                    directionTrend = 0;
                    topPointTrend = 0;
             	}
                directionNum = 0;
          	}
          }
        },0,990); //run every defined ms
    }
    
    private void writeCommand() {
  		out1[0] = out[0];
  		if ((char)out[0]>='a' && (char)out[0]!='o') {
  	  	    out1[0] = (byte)(commandsMap.get((char)out[0]) & 0xFF);
  		}
  		if ((char)out1[0]!='o') {
  	     	MainActivity.mSerialService.write(out1);
  		}
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
    	else if (command=='o') txtCommand = "load";
    	else if (command=='t') txtCommand = "turn";
		else if (command=='q') txtCommand = "q";
		else if (command=='e') txtCommand = "e";
    	else if (command=='$') txtCommand = "finish";
    	else if (command=='n') txtCommand = "new azimuth";
    	else if (command>='0' && command<='9') txtCommand = ""+command;
    	if (txtCommand!="") {
    		if (searchMode<1 || searchMode>3 || mCommand!='w' || mPrevCommand!=mCommand) {
    			long time = System.currentTimeMillis()/1000;
            	if (voiceOutput>0 && time!=lastSay) {
            		lastSay = time;
            		say(txtCommand);
            	}
        		//Toast.makeText(getApplicationContext(), txtCommand, Toast.LENGTH_SHORT).show();
    		}
    	}
    }
    
    private void say(String txt) {
    	if (voiceOutput>0 && repeatedCommand<1) tts.speak(txt, TextToSpeech.QUEUE_ADD, null);	
    	if (txt=="stop") repeatedCommand = 1;
    	else repeatedCommand = 0;
    }
    
    private void saveConfig() {
        Toast.makeText(getApplicationContext(), "saving...", Toast.LENGTH_SHORT).show();
        mPrefs = PreferenceManager.getDefaultSharedPreferences(this);
    	SharedPreferences.Editor editor = mPrefs.edit();
    	editor.putString(SEARCHMODE_KEY, Integer.toString(searchMode));
    	editor.putString(THRESHOLDLIMIT_KEY, Integer.toString(mLevel));
    	editor.putString("btdevice", address);
    	editor.putString("btdevice2", address2);
    	editor.putString("limit1", ""+limit1);
    	editor.putString("minarea", ""+mArea);
    	editor.putString("hsv1", ""+fileNumber);
    	editor.putString("hsv2", mHSV2);
    	editor.commit();
    }

    private void readPrefs() {
        mPrefs = PreferenceManager.getDefaultSharedPreferences(this);
		limit1 = readIntPref("limit1", limit1, 5);
		limit2 = limit1 + 10;
		azimuthLimit = readIntPref("azimlimit", azimuthLimit, 25);
		mArea = readIntPref("minarea", mArea, 9999);
		//if (mArea>10) minArea = mArea/100; 
		fileNumber = readIntPref("hsv1", fileNumber, 99);
		mHSV2 = mPrefs.getString("hsv2", mHSV2);
		mHSV1 = mHSV2;
		mBlobColorHsv1 = setColorFromString(mHSV1,mBlobColorHsv1);
		mBlobColorHsv2 = setColorFromString(mHSV2,mBlobColorHsv2);
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
    
    private Scalar setColorFromString(String mStr, Scalar mDefault) {
    	Scalar mReturn = mDefault;
		boolean ok = true;
		double[] mColors = new double[4];
		int ix = 0;
    	if (mStr.length()>10) {
    		try {
        		mStr = mStr.replace("[","");
        		mStr = mStr.replace("]","");
        		String[] mNums = mStr.split(",");
        		for (ix=0; ix<mNums.length; ix++) {
        			mColors[ix] = Double.parseDouble(mNums[ix]);
        		}
            } catch (NumberFormatException e) {
                ok = false;
            	Toast.makeText(getApplicationContext(), "setColorFromString error "+mStr+", "+ix+", "+e.toString(), Toast.LENGTH_SHORT).show();
    		}
    		if (ok) mReturn.set(mColors);
    	}
    	//Toast.makeText(getApplicationContext(), ""+mStr+", "+ok+", "+mDefault.toString(), Toast.LENGTH_SHORT).show();
    	return mReturn;
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

    public static void appendLog(String text)
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

    public void appendToFile(String file, String text)
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

    public void getLocation()
    {
     // Get the location manager
     //LocationManager locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
     //Criteria criteria = new Criteria();
     //String bestProvider = locationManager.getBestProvider(criteria, false);
     Location location = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
     //if (location==null) location = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
//     Location location  = LocationServices.FusedLocationApi.getLastLocation(
//             mGoogleApiClient);
     lat = -1.0;
     lon = -1.0;
     accuracy = 999;
     bearing = 999;
     stav = "ERR";
     now.setToNow();
     if (location!=null && location.getTime()>(now.toMillis(false)-5000)) {
         lat = location.getLatitude();
         lon = location.getLongitude();
         accuracy = location.getAccuracy();
         bearing = location.getBearing();
         stav = "OK";
         lastGPStime = location.getTime();
         //return new LatLng(lat, lon);
     } else {
    	 location = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
         if (location!=null && location.getTime()>(now.toMillis(false)-30000)) {
             lat = location.getLatitude();
             lon = location.getLongitude();
             accuracy = location.getAccuracy();
             bearing = location.getBearing();
             stav = "NET";
             lastNETtime = location.getTime();
         }
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
        		wpMode0 = wpMode;
        		wp += step;
        		wpLat = path.get(wp).y;
        		wpLon = path.get(wp).x;
        		wpMode = wpModes.get(wp)[0];
        		mGrass = 0;
        		if (wpMode>1 || wpMode0>1) mGrass = 1;
        		float[] results = new float[3];
        		Location.distanceBetween(wpLat0, wpLon0, wpLat, wpLon, results);
        		wpDist = results[0];
        		pathAzimuth = results[1];
        		//azimuthToNextWaypoint = results[2];
            	lastDist = btDst;
            	if (searchMode!=1) mTargetAzimuth = (int)pathAzimuth;
    		}
            catch (Exception e) {
                e.printStackTrace();                    
            }
    	}
    }

    public List<Point> readPoints(String fil)
    {
    	List<Point> ret = new ArrayList<Point>();
		if (fil.contains("Path")) wpModes.clear();
		else if (fil.contains("Goal")) goalPoints.clear();
        try {
            File sdcard = Environment.getExternalStorageDirectory();
            File file = new File(sdcard,fil);
            if (file.exists()) {
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
                		if (tmp.length>3) if (tmp[3].length()>0) xMode0 = Integer.parseInt(tmp[3]);
                		int[] xMode = {xMode0};
                		if (fil.contains("Path")) wpModes.add(xMode);
                		else if (fil.contains("Goal")) goalPoints.add(xMode);
                		Point tmpPoint = new Point(xLon,xLat);
                    	ret.add(tmpPoint);
                	}
                }
                br.close();
            }
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
            if (!file.exists()) {
                file = new File(sdcard,"RoboNavMap.txt");
            }
            points.clear();
            edges.clear();
        	minLat = 999.0;
        	minLon = 999.0;
        	maxLat = -999.0;
        	maxLon = -999.0;
            if (file.exists()) {
                BufferedReader br = new BufferedReader(new FileReader(file));  
                String line;
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
            }
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

    static public List<String> readRobots(String fil)
    {
    	List<String> ret = new ArrayList<String>();
    	allowedAddresses = "";
		robots.clear();
        try {
            File sdcard = Environment.getExternalStorageDirectory();
            File file = new File(sdcard,fil);
            BufferedReader br = new BufferedReader(new FileReader(file));  
            String line;
            while ((line = br.readLine()) != null) {
            	if (line.length()>10) {
                	ret.add(line.trim());
    	            //Toast.makeText(getApplicationContext(), "line: "+line, Toast.LENGTH_SHORT).show();
        	    	String[] parts = line.trim().split(";");
        	    	allowedAddresses += " "+parts[0].trim();
        	    	if (connectedAddress.equalsIgnoreCase(parts[0].trim())) {
        	    		robotName = parts[1];
        	            //Toast.makeText(getApplicationContext(), "robot: "+robotName, Toast.LENGTH_SHORT).show();
        	    		if (parts[2].length()>=commandsTable0.length()) {
        		    		commandsTable = parts[2];
        		    		commandsMap.clear();
        		    		for (int ix=0; ix<commandsTable0.length(); ix++) {
        		    			commandsMap.put(commandsTable0.charAt(ix), commandsTable.charAt(ix));
        		    		}
        	    		}
        	    		if (parts[3].length()>0) {
        		    		//testTxt += ":"+parts[3];
        		    		telemetryTable = parts[3];
        		    		//testTxt += ":"+telemetryTable.length();
        		    		parseTelemetryTable();
        	    		}
        	    	}
            	}
            }
            br.close();
        }
        catch (Exception e) {
            e.printStackTrace();                    
        }
        return ret;
    }

    public void computePosition()
    {
    	compassAzimuth = compass.getAzimuth();
    	latOK = lat;
    	lonOK = lon;
		azimuthValid = 0;
	    if (compassAzimuth!=999) {
			azimuthOK = compassAzimuth;
			azimuthValid = 1;
	    }
    	if (btValid>0) {
        	if (searchMode!=22 || azimuthValid<1) {
        		azimuthOK = azimuth;
    			azimuthValid = 1;
        	}
        	distanceOK = btDst;
        	if (btLat>30f) {
        		// GPS at robot is better
            	latOK = btLat;
            	lonOK = btLon;
        	}
    	} else if (stav=="OK") {
    		// azimuth from GPS?
    		if (bearing!=0 && azimuthValid<1) {
            	azimuthOK = (bearing + 360) % 360;
    			azimuthValid = 1;
    		}
    	}
    	if (azimuthValid<1) {
    		// azimuth from path
        	azimuthOK = pathAzimuth;
    		azimuthValid = 1;
    	}
    	azimuthOK += azimuth_calib;
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
    	btRngL = 99; // center sonar range from bluetooth
    	btRngR = 99; // center sonar range from bluetooth
    	btDst = 0; // distance from bluetooth
    	btSpd = 0; // speed from bluetooth
    	btRngLeft = 99; // left sonar range from bluetooth
    	btRngRight = 99; // right sonar range from bluetooth
    	btRngBack = 99; // IR range from bluetooth
    	btRngLeftL = 99; // left sonar range from bluetooth
    	btRngRightL = 99; // right sonar range from bluetooth
    	btRngLeftR = 99; // left sonar range from bluetooth
    	btRngRightR = 99; // right sonar range from bluetooth
    	wp = 0; // waypoint number
    	runMode = 0; // run mode (0=wait_for_start, 1=run, 2=finish)
    	goalReached = 0; // we should be at start now
    	state = "normal";
    	parseTelemetryTable();
        appendLog("init");
}

    public void computeCommand()
    {
    	// main navigation algorithm (finite automata and fuzzy logic)
    	whatToDo(); // get current state
    	if (state=="normal") {
    		if (searchMode!=4) {
            	getAngleToNextWaypoint();
            	getAngleToStayOnRoad();
            	getAngletoAvoidObstacle();
            	turnByDrivingSignals();
    		} else {
            	//getAngleToNextWaypoint();
            	//getAngleToStayOnRoad();
            	//getAngletoAvoidObstacle();
            	trackObject();
            	//turnByDrivingSignals();
    		}
    	}
    }
    
    public void whatToDo() {
        mod2 = mLevel % 2;
        mod6 = mLevel % 6;
        mText += ""+mod6;
		Location.distanceBetween(latOK, lonOK, wpLat, wpLon, results);
		distanceToNextWaypoint = results[0];
		azimuthToNextWaypoint = results[1];
    	if (latOK<33f && searchMode!=1 && runMode==1) {
    		distanceToNextWaypoint = 999;
    		wpDist1 = 999;
    		pathAzimuth = azimuthOK;
    		azimuthToNextWaypoint = azimuthOK;
    	}
    	//turnAngleToNextWaypoint = pathAzimuth - azimuthToNextWaypoint; // actual azimuth difference
    	turnAngleToNextWaypoint = azimuthToNextWaypoint - azimuthOK; // actual azimuth difference
    	if (turnAngleToNextWaypoint<-180) turnAngleToNextWaypoint += 360;
    	else if (turnAngleToNextWaypoint>180) turnAngleToNextWaypoint -= 360;
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
    	 * - turnAngleToNextWaypoint (actual azimuth difference to the line between the actual position and the next waypoint)
    	 * - actualDist (actual distance from the last waypoint)
    	 * - wpDist (distance from last waypoint to the next waypoint)
    	 * - btRng, btRngLeft, btRngRight (ranges to obstacles ahead, left and right)
    	 * - btValid (1 => data from BT link are valid, 0 = BT data are not valid)
    	 * - azimuthValid (1 => measured azimuth is valid, 0 = azimuth is not measured)
    	 * - state (actual state of the algorithm)
    	 * - distanceToNextWaypoint (distance to the next waypoint)
    	 * - wp (number of the next waypoint)
    	 * - cameraDirection (mean direction between averageDirection and averageTopPoint)
    	 * - cameraDiff (difference between averageDirection and averageTopPoint)
    	 * - cameraProbability (probability of the accuracy of the cameraDirection)
    	 * - azimuthLimit, limit1, limit2 (limits of the azimuth difference)
    	 * - leftOK, rightOK (position of road edge on the left and right (-2=image_center +2=image_limit)
    	 * - roadDetected (probability of the road ahead)
    	 * 
    	 * strategy for RobotemRovne:
    	 * - drive by initial azimuth
    	 * - if best estimated roadDirection is off driving azimuth, slightly adjusti tagetAzimuth
    	 * - if obstacle left or right is close, adjust targetAzimuth to oposite direction
    	 * 
    	 * strategy for RoboOrienteering:
    	 * - drive to next waypoint by azimuth
    	 * - near of dropping waypoint switch navigation to orange cone (if is in sight)
    	 * - if height of cone is above limit, drop ball, set next waipoint and avoid orange cone
    	 * 
    	 * strategy for RoboTour:
    	 * - start with loading procedure (automatic paylouad pick up)
    	 * - turn to targetAzimuth and avoid any obstacle (another robots)
    	 * - drive to next waypoint by azimuth, but on the road only
    	 * - prefer best estimation of roadDirection (topDirection, direction, topHeight, azimuthOK, azimDiff, leftOK, rightOK)
    	 * - if crossing is near, switch to wall follow procedure (to safely turn to next targetAzimuth) by leftOK and rightOK
    	 * - at finish waypoint drop payload and then navigate back to start (stop at start)
    	 */
		mCommand = '-'; // implicit mCommand
		if (searchMode==4) {
			// Road Assistance
			state = "normal";
			//mCommand = 'w';
			return;
		}
		if (searchMode>0) {
			// finite automata navigation
	  		//mCommand = '-'; // implicit mCommand
  			actualDist = distanceOK - lastDist;
  			// step backward if state was unknown;
  			if (state=="unknown") {
  				state = "back";
  				mCommand = 'b';
  				return;
  			}
  			else if (state=="back") {
  				//state = "back2";
  				//mCommand = 'b';
  				state = "back3";
  				mCommand = 's';
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
	  		if (wpDist>0 && wpDist<999 && searchMode>1 && state!="end") {
	  			// if orange cone is close (at RO), then drop payload (waypoint is reached)
	  			if (searchMode==2 && wpMode>0 && distanceToNextWaypoint<15 && state!="drop" && ((mBoundingRectangle.height>h/4) || (mBoundingRectangle.height>h/5 && mBoundingRectangle.y>(h/2)))) {
	  				state = "drop";
	  				tmpSec0 = 0;
	  				say("cone");
	  				mText += "cone";
	  			}
	  			if ((actualDist>(wpDist+30) && mBoundingRectangle.height<10) || (distanceToNextWaypoint<7 && mBoundingRectangle.height<10) || (distanceToNextWaypoint<51 && Math.abs(turnAngleToNextWaypoint)>110) || state=="drop") {
	  				// waypoint reached
	  				mText += "WP"+wp;
	  				if (state!="drop") say("point "+wp);
	  		    	if (path.size()==(wp+1) || wp==0 || wpMode>0 || state=="drop") {
	  		    		// payload drop
	  		    		if (tmpSec0==0) {
	  		    			// start payload drop procedure
	  		    			state = "drop";
	  		    			tmpSec0 = tmpSec;
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
		  		    		mCommand = 's'; // stop
		  		    		counter = 0;
		  		    		return;
	  		    		} else if ((tmpSec0-tmpSec)>9) {
	  		    			// end of payload drop procedure
	  		    			state = "normal";
	  		    			tmpSec0 = 0;
		  		    		mCommand = 'w'; // forward
		  		    		return;
		  		    	} else if (searchMode==3) {
		  		    		// Robotour
	  		    			state = "drop";
  		  		    		mCommand = 's'; // stop
		  				    if ((tmpSec0-tmpSec)%4==1) {
		  		  		    	if (wpMode0>1) {
		  		  		    		mCommand = 'p'; // payload drop
		  		  		    	} else {
		  		  		    		mCommand = 'o'; // payload load
		  		  		    		//mCommand = 's'; // stop
		  		  		    	}
		  				    }
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=1) {
	  		    			state = "drop";
	  		    			if (wpMode0>1) {
			  		    		mCommand = 'p'; // payload drop
	  		    			} else {
			  		    		mCommand = 's'; // stop
	  		    			}
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=2) {
	  		    			// RoboTour => stop
	  		    			state = "drop";
		  		    		mCommand = 's'; // stop
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=3) {
	  		    			state = "drop";
		  		    		mCommand = 'r'; // right
		  		    		//if (blobDirection>30) mCommand = 'l';
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=4) {
	  		    			state = "drop";
		  		    		mCommand = 'r'; // right
							//mCommand = 'w'; // forward (test 12.6.2018)
		  		    		//if (blobDirection>30) mCommand = 'l';
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=5) {
	  		    			state = "drop";
		  		    		mCommand = 'r'; // right
		  		    		//mCommand = 'w'; // forward
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=6) {
	  		    			state = "drop";
		  		    		mCommand = 'w'; // forward
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=7) {
	  		    			state = "drop";
		  		    		mCommand = 'l'; // left
		  		    		if (turnAngleToNextWaypoint>20) mCommand = 'r';
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))<=8) {
	  		    			state = "drop";
		  		    		mCommand = 'l'; // left
		  		    		if (turnAngleToNextWaypoint>20) mCommand = 'r';
		  		    		return;
	  		    		} else if (((tmpSec0-tmpSec))>=9) {
	  		    			state = "normal";
		  		    		mCommand = 'w'; // forward
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
		}
		
    }

    public void getAngleToNextWaypoint() {
		// navigation to the next waypoint
		Location.distanceBetween(latOK, lonOK, wpLat, wpLon, results);
		distanceToNextWaypoint = results[0];
		wpDist1 = distanceToNextWaypoint;
		azimuthToNextWaypoint = results[1];
    	if (latOK<33f && searchMode!=1 && runMode==1) {
    		distanceToNextWaypoint = 999;
    		wpDist1 = 999;
    		pathAzimuth = azimuthOK;
    		azimuthToNextWaypoint = azimuthOK;
    	}
    	azimDiff = pathAzimuth - azimuthOK; // initial azimuth difference
    	if (mTargetAzimuth!=999 && (searchMode==1 || searchMode==5)) azimDiff = azimuthOK - mTargetAzimuth; // azimuth difference
    	if (azimDiff<-180) azimDiff += 360;
    	else if (azimDiff>180) azimDiff -= 360;
    	turnAngleToNextWaypoint = azimuthToNextWaypoint - azimuthOK; // actual azimuth difference
    	if (turnAngleToNextWaypoint<-180) turnAngleToNextWaypoint += 360;
    	else if (turnAngleToNextWaypoint>180) turnAngleToNextWaypoint -= 360;
    }

    public void getAngleToStayOnRoad() {
//		// new procedure (since 2016-06-02)
//		if (searchMode==1) {
//			if (mBoundingRectangle.height>9) {
//				if (blobDirection<=0 && blobDirection>-15) drivingSignal = 30;
//				else if (blobDirection>0 && blobDirection<15) drivingSignal = -30;
//			}
//		}
//        
//		if (searchMode>1) {
//			if (searchMode!=3 && mBoundingRectangle.height>9) {
//				if (distanceToNextWaypoint<12) {
//					drivingSignal = blobDirection;
//					if (mBoundingRectangle.height>110 && wpMode>0) {
//						state = "drop";
//						mCommand = 's';
//					}
//				}
//				else if (blobDirection<=0 && blobDirection>-15) drivingSignal = 30;
//				else if (blobDirection>0 && blobDirection<15) drivingSignal = -30;
//			}
//			else if ((searchMode!=3 && mGrass>=0) || mCenterOK<1) {
//				drivingSignal = turnAngleToNextWaypoint/2;
//			}
//			else if (Math.abs(turnAngleToNextWaypoint)>45) {
//				drivingSignal = turnAngleToNextWaypoint/2;
//			}
//		}
		// probabilistic road navigation
		if ((searchMode>=1 || mBoundingRectangle.height<10) && roadSearch>=0) {
	  		if (directionNum>=0) {
	  			direction = averageDirection;
	  			topDirection = averageTopPoint;
	  		}
	  		if (btValid>0 && btDst<20 && searchMode<1) {
	  			// stabilization for RR
	  			direction /= 3-btDst/10;
	  			topDirection /= 3-btDst/10;
	  		}
	  		cameraDirection = (2*direction + 3*topDirection) / 5;
	  		cameraDiff = Math.abs(direction - 2*topDirection);
	  		cameraProbability = 100 - (2*cameraDiff);
	  		if (topHeight<2) cameraProbability = 0; // blob is too close, camera is not reliable
	  		azimDiffOK = (int)(-azimuthValid * azimDiff);
	  		if (searchMode==1 || searchMode==2) mCommand = '-'; // delete previously computed command (set default command to none for RR)
			if (mod6==2 || mod6==3 || mod6==5) {
  			    // HSV saturation mode only
  				mText += "S";
			}
			if (mod6==0 || mod6==2 || mod6==4) {
  			    // navigation by topDirection
				//drivingSignal = cameraDirection;
				directionOK = topDirection;
  				mText += "t";
			} else if (mod6==1 || mod6==3 || mod6==5) {
  			    // navigation by camera direction
				directionOK = cameraDirection;
				//drivingSignal = topDirection;
  				mText += "ca";
			} else {
				// navigation by camera and compass
  				mText += "mix";
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
  	    	//if (searchMode==1 && mod6<6) {
  	    	//	turnAngleToStayOnRoad += azimDiff/1;
  	    	//}
  			turnAngleToStayOnRoad = directionOK;
		}
    }

    public void getAngletoAvoidObstacle() {
		// obstacle avoiding
		if (btValid>0) {
			if (btRngLeft<7 || btRngLeftL<7 || btRngLeftR<7) {
				// left obstacle
				//mCommand = 'r';
	    		//turnAngleToAvoidObstacle = 40.0;
			}
			if (btRngRight<7 || btRngRightL<7 || btRngRightR<7) {
				// right obstacle
				//mCommand = 'l';
	    		//turnAngleToAvoidObstacle = -40.0;
			}
			if (searchMode!=1 && (btRng<4 || btRngL<4 || btRngR<4)) {
				// front obstacle
				if (avoiding<1) {
		    		mCommand = 's'; // stop
		    		turnAngleToAvoidObstacle = -180.0;
		    		avoiding = 1;
				} else if (btRngBack>30) {
		    		mCommand = 'b'; // back
		    		turnAngleToAvoidObstacle = 180.0;
				} else {
		    		mCommand = 's'; // stop
		    		turnAngleToAvoidObstacle = -180.0;
				}
			}
			else if (avoiding>0) {
				avoiding = 0;
				mCommand = 'w'; // forward
	    		turnAngleToAvoidObstacle = 0.0;
			}
		} 
    }

    public void turnByDrivingSignals() {
    	// driving signals:
    	// - distanceToNextWaypoint, turnAngleToNextWaypoint
    	// - turnAngleToStayOnRoad, direction, topDirection, directionOK
    	// - avoiding, turnAngleToAvoidObstacle

    	if (searchMode==3) {
			// RoboTour (simple algorithm since 2017-12-30)
			mText += "RT";
			// drive to waypoint
			if (turnAngleToNextWaypoint < -limit2) mCommand = 'l'; // extra left
			else if (turnAngleToNextWaypoint <= -limit1) mCommand = 'h'; // slightly left
			else if (turnAngleToNextWaypoint > limit2) mCommand = 'r'; // extra right
			else if (turnAngleToNextWaypoint >= limit1) mCommand = 'k'; // slightly right
			// stay on road
			drivingSignal = directionOK;
			if (mod6 <= 1) {
				// by direction to blob centroid
				drivingSignal = direction;
				mText += "c";
			} else if (mod6 <= 3) {
				// by topDirection
				drivingSignal = topDirection;
				mText += "t";
			}
			if (drivingSignal < -limit2) mCommand = 'l'; // extra left
			else if (drivingSignal <= -limit1) mCommand = 'h'; // slightly left
			else if (drivingSignal > limit2) mCommand = 'r'; // extra right
			else if (drivingSignal >= limit1) mCommand = 'k'; // slightly right
			// avoid obstacle
			if (avoiding > 0 || Math.abs(turnAngleToAvoidObstacle) > 1.0) {
				// avoiding, so drive by turnAngleToAvoidObstacle
				mText += "avo";
				if (turnAngleToAvoidObstacle < -170.0) mCommand = 's';
				else if (turnAngleToAvoidObstacle > 170.0) mCommand = 'b';
				else if (turnAngleToAvoidObstacle > 30.0) mCommand = 'r';
				else if (turnAngleToAvoidObstacle < -30.0) mCommand = 'l';
			}
		} else if (searchMode==1) {
    		// Robotem rovne (since 2018-05-09)
			// 1. stay on road
			//drivingSignal = turnAngleToStayOnRoad;
            if (mod6<2) {
                drivingSignal = topDirection;
                mText += "T";
            } else if (mod6<4) {
                directionOK = (int)Math.round(topDirection * 0.5 + cameraProbability * cameraDirection * 0.005);
                drivingSignal = directionOK;
                mText += "X";
            } else {
                drivingSignal = cameraDirection;
                mText += "O";
            }

			if ((mod6%2)==1) {
				drivingSignal += azimDiff/2;
				mText += "C";
			}

			if (drivingSignal<-limit2) mCommand = 'l'; // extra left
			else if (drivingSignal<=-limit1) mCommand = 'h'; // slightly left
			else if (drivingSignal>limit2) mCommand = 'r'; // extra right
			else if (drivingSignal>=limit1) mCommand = 'k'; // slightly right

			// 4. sanitize command (restrictions and stabilization)
			if (" kr".indexOf(mCommand)>0 && mRightOK<1) mCommand = 'w'; // can't turn right
			else if (" hl".indexOf(mCommand)>0 && mLeftOK<1) mCommand = 'w'; // can't turn left
			if (mPrevCommand=='b' && mCommand!='b') {mCommand = 's'; state="stop";}
			if (mCommand=='b' && mPrevCommand!='b' && mPrevCommand!='s') mCommand = 's';
			if ((searchMode==1 || searchMode>=2) && mCommand=='-') mCommand = 'w'; // normal mCommand for RR & RO is "forward"
			if (" lhkr-".indexOf(""+mCommand)>0 && state=="normal" && btSpd<1 && btValid>0) mCommand = 'w'; // forward (if speed is 0)
			if (mCommand=='w' && state=="stop") state = "normal";
			if (searchMode>=1) {
				// stabilization
				numCleanCommands++;
				if (mCommand!='w') {
					// stabilize only few times and only if course is almost OK
					//if (numCleanCommands<3 && Math.abs(turnAngleToNextWaypoint)<30.0) mCommand = 'w';
					if (numCleanCommands<3 && Math.abs(azimDiff)<30.0) mCommand = 'w';
					else {
						numCleanCommands = 0;
						if (btSpd>160 || searchMode>1 ) numCleanCommands = 1;
					}

				}
			}
    	} else {
        	// since 2017-06-20
        	// 1. stay on road
      		if (mod6<3 && searchMode!=1) {
    			// PD navigation by computed driving signal
    			drivingSignal = directionOK - 0.2 * (directionOK - lastDirectionOK);
    			lastDirectionOK = directionOK;
    			mText += "PD";
    		} else {
    			// direct navigation
    			drivingSignal = turnAngleToStayOnRoad;
    			mText += "D";
      		}

      		// 2. set drivingSignal by turnAngleToNextWaypoint
      		if (searchMode==2) drivingSignal = turnAngleToNextWaypoint;
      		else if ((turnAngleToNextWaypoint>limit2 && drivingSignal>-limit2 && mRightOK>0) || (turnAngleToNextWaypoint<-limit2 && drivingSignal<limit2 && mLeftOK>0)) {
      	    	if (searchMode!=1) drivingSignal = turnAngleToNextWaypoint/2;
      		}
        	
        	if (searchMode==1 && mod6>=4) {
        		drivingSignal += azimDiff/2;
    			mText += "C";
        	}

        	// 3. avoid obstacle
      		if (avoiding>0 || Math.abs(turnAngleToAvoidObstacle)>1.0) {
    			// avoiding, so drive by turnAngleToAvoidObstacle
    			mText += "avo";
                if (turnAngleToAvoidObstacle<-170.0) mCommand = 's';
    			else if (turnAngleToAvoidObstacle>170.0) mCommand = 'b';
    			else if (turnAngleToAvoidObstacle>30.0) mCommand = 'r';
    			else if (turnAngleToAvoidObstacle<-30.0) mCommand = 'l';
    		} else {
    			// not avoiding, so drive by drivingSignal
                if (drivingSignal<-limit2) mCommand = 'l'; // extra left
    			else if (drivingSignal<=-limit1) mCommand = 'h'; // slightly left
    			else if (drivingSignal>limit2) mCommand = 'r'; // extra right
    			else if (drivingSignal>=limit1) mCommand = 'k'; // slightly right

                // long range obstacle avoiding (for RR, RO and RT)
        		if (searchMode>=1 && (mod6==1 || mod6==5)) {
        			mText += "S";
        			if (btValid>0 && btRng>18 && btRngLeft<18 && btRngRight>18) {
        				//mCommand = 'k'; // slightly right
        				mCommand = 'r'; // right
        			} else if (btValid>0 && btRng>18 && btRngLeft>18 && btRngRight<18) {
        				//mCommand = 'h'; // slightly left
        				mCommand = 'l'; // left
        			}
        		}
            	
    			if (mGrass<0 && mCenterOK>0) {
    			    if (" kr".indexOf(mCommand)>0 && mRightOK<1) mCommand = 'w'; // can't turn right
    			    else if (" hl".indexOf(mCommand)>0 && mLeftOK<1) mCommand = 'w'; // can't turn left
    		    }
    		}

       	    // navigation to orange cone
    		if (wpMode>0 && searchMode==2 && distanceToNextWaypoint<19 && state!="drop" && mBoundingRectangle.height>(h/100)) {
      			int coneDirection = blobDirection;
      			//if (mBoundingRectangle.height>(h/8)) coneDirection += 20;
    			if (coneDirection<-30) mCommand = 'l';
    			else if (coneDirection<-10) mCommand = 'h';
    			else if (coneDirection>30) mCommand = 'r';
    			else if (coneDirection>10) mCommand = 'k';
    			else mCommand = 'w';
    			mText += "cone";
    		}

      		// 4. sanitize command (restrictions and stabilization)
     		if (searchMode>1 && (mod6==1 || mod6==5)) {
     			// check L/R sonars
    			//mText += "S";
     	    	if (" lh".indexOf(""+mCommand)>0 && btRngLeft<10) mCommand = 'w'; // can't turn left
     	    	else if (" kr".indexOf(""+mCommand)>0 && btRngRight<10) mCommand = 'w'; // can't turn right
     			if (btValid>0 && searchMode!=1) {
     				if (btRngLeft<14 || btRngLeftR<14 || btRngL<14) {
     					// left obstacle
     					mText += "lo";
     					if (" lhw".indexOf(""+mCommand)>0 && btValid>0) mCommand = 'r';
     				}
     				if (btRngRight<14 || btRngRightL<14 || btRngR<14) {
     					// right obstacle
     					mText += "ro";
     					if (" rkw".indexOf(""+mCommand)>0 && btValid>0) mCommand = 'l';
     				}
     			} 
     		}
    		if (" kr".indexOf(mCommand)>0 && mRightOK<1) mCommand = 'w'; // can't turn right
    		else if (" hl".indexOf(mCommand)>0 && mLeftOK<1) mCommand = 'w'; // can't turn left
    		if (searchMode>1 && (mod6==2 || mod6==5)) {
    			mText += "U";
    	  		if (" wlhkr-".indexOf(mCommand)>0 && (topHeight<2 || mCenterOK<1)) {
    				// road border seems too close, rather stop and step back (not for RR)
        			state = "unknown";
        			mCommand = 's'; // stop
    	  		}
      		}
    		if (mPrevCommand=='b' && mCommand!='b') {mCommand = 's'; state="stop";}	
        	if (mCommand=='b' && mPrevCommand!='b' && mPrevCommand!='s') mCommand = 's';
        	//if (mCommand=='b' && mPrevCommand=='s') mCommand = 'f';
    		if ((searchMode==1 || searchMode>=2) && mCommand=='-') mCommand = 'w'; // normal mCommand for RR & RO is "forward"
        	//if ((" lhkr-".indexOf(""+mCommand)>0 && (state=="normal" && mPrevCommand=='s')) || state=="stop") mCommand = 'f'; // straight
        	if (" lhkr-".indexOf(""+mCommand)>0 && state=="normal" && btSpd<1 && btValid>0) mCommand = 'w'; // forward (if speed is 0)
        	if (mCommand=='w' && state=="stop") state = "normal";
    		if (searchMode>=1) {
    			// stabilization
    			numCleanCommands++;
    			if (mCommand!='w') {
    				// stabilize only few times and only if course is almost OK
    				//if (numCleanCommands<3 && Math.abs(turnAngleToNextWaypoint)<30.0) mCommand = 'w';
    				if (numCleanCommands<3 && Math.abs(azimDiff)<30.0) mCommand = 'w';
    			    else {
    			    	numCleanCommands = 0;
    			    	if (btSpd>160 || searchMode>1 ) numCleanCommands = 1;
    			    }
    				
    			}
    		}
    	}
    	
    }

    
    public void slowCommand() {
    	out[0] = mCommand;
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
    }

    public void fastCommand() {
    	out[0] = mCommand;
     	writeCommand();
    	tellCommand((char)out[0]);
     	mPrevCommand = mCommand;
     	mCommand = '-';
    }

    public void trackObject() {
		// color blob navigation
		mCommand = 's';
		drivingSignal = 0.0;
		if (mBoundingRectangle!=null) {
			if (mBoundingRectangle.height>(h/50)) {
				if (blobDirection<-limit2) mCommand = 'l';
				else if (blobDirection<-limit1) mCommand = 'h';
				else if (blobDirection>limit2) mCommand = 'r';
				else if (blobDirection>limit1) mCommand = 'k';
				else if (mBoundingRectangle.br().y>(h-h/4)) mCommand = 'b';
				else if (mBoundingRectangle.br().y<(h/2) || mBoundingRectangle.height<(h/40)) mCommand = 'w';
				drivingSignal = (double)blobDirection;
			}
		}
    }

}
