package cz.vmoste.robonav;

import java.util.List;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
//import org.opencv.core.Size;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import cz.vmoste.robonav.R;
import android.app.Activity;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnTouchListener;

public class ColorBlobDetectionActivity extends Activity implements OnTouchListener, CvCameraViewListener2 {
    private static final String  TAG              = "OCVSample::Activity";

	private static String tmpText = "";
	private static String toRgba = "none";
	
	private TextToSpeech tts;
	
    private boolean              mIsColorSelected = false;
    private Mat                  mRgba;
    private Scalar               mBlobColorRgba;
    private Scalar               mBlobColorHsv;
    private ColorBlobDetector    mDetector;
    private Mat                  mSpectrum;
    //private Size                 SPECTRUM_SIZE;
    private Scalar               CONTOUR_COLOR;
    private byte                 command = '-';
    private byte                 prevCommand = '-';
    private byte[]               out = new byte[1];
	private int w = 300;
	private int h = 200;
	private int p1 = 100;
	private int p2 = 200;
	private double siz = 0.8;
	private int pos = 100;
	private int wi = 1;
	
	private static int searchMode = 0;

    private CameraBridgeViewBase mOpenCvCameraView;

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    mOpenCvCameraView.enableView();
                    mOpenCvCameraView.setOnTouchListener(ColorBlobDetectionActivity.this);
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };

    public ColorBlobDetectionActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        super.onCreate(savedInstanceState);
        tts = new TextToSpeech(getApplicationContext(), 
        	      new TextToSpeech.OnInitListener() {
        	      @Override
        	      public void onInit(int status) {
        	         if(status != TextToSpeech.ERROR){
        	             tts.setLanguage(Locale.US);
        	            }				
        	         }
        	      });
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.color_blob_detection_surface_view);

        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);
        sendCommand(); // start timer
    }

    @Override
    public void onPause()
    {
        if(tts !=null){
            tts.stop();
            tts.shutdown();
         }
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
        //if (screenWakeLock != null) {
        //	   if(screenWakeLock.isHeld())
        //	      screenWakeLock.release();
        //	   screenWakeLock = null;
      	//}
    }

    public void onDestroy() {
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mDetector = new ColorBlobDetector();
        mSpectrum = new Mat();
        mBlobColorRgba = new Scalar(255);
        mBlobColorHsv = new Scalar(255);
        //SPECTRUM_SIZE = new Size(200, 64);
        CONTOUR_COLOR = new Scalar(255,0,0,255);
    	//params = getWindow().getAttributes();
        //params.screenBrightness = 0.0f;
    	//getWindow().setAttributes(params);
        // autotouch (green)
        //Toast.makeText(getApplicationContext(), "autotouch", Toast.LENGTH_SHORT).show();
        //mDetector.setColorRadius(new Scalar(25,50,50,0));
        mBlobColorHsv.val[0] = 100;
        mBlobColorHsv.val[1] = 200;
        mBlobColorHsv.val[2] = 70;
        mBlobColorHsv.val[3] = 0;
        mBlobColorRgba = converScalarHsv2Rgba(mBlobColorHsv);
        mDetector.setHsvColor(mBlobColorHsv);
        mIsColorSelected = true;
    	w = mRgba.width();
    	h = mRgba.height();
    	p1 = w/3;
    	p2 = p1*2;
    	pos = h - 10;
    	siz = 0.7;
    	if (w>400) {
    		siz = 1.4;
    		wi = 2;
    	}
    }

    public void onCameraViewStopped() {
        mRgba.release();
    }

    public boolean onTouch(View v, MotionEvent event) {
        int cols = mRgba.cols();
        int rows = mRgba.rows();

        int xOffset = (mOpenCvCameraView.getWidth() - cols) / 2;
        int yOffset = (mOpenCvCameraView.getHeight() - rows) / 2;

        int x = (int)event.getX() - xOffset;
        int y = (int)event.getY() - yOffset;

        Log.i(TAG, "Touch image coordinates: (" + x + ", " + y + ")");

        if ((x < 0) || (y < 0) || (x > cols) || (y > rows)) return false;

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

        mDetector.setHsvColor(mBlobColorHsv);

        //Imgproc.resize(mDetector.getSpectrum(), mSpectrum, SPECTRUM_SIZE);

        mIsColorSelected = true;

        touchedRegionRgba.release();
        touchedRegionHsv.release();

        return false; // don't need subsequent touch events
    }

    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
        mRgba = inputFrame.rgba();

        if (mIsColorSelected) {
            mDetector.process(mRgba);
            List<MatOfPoint> contours = mDetector.getContours();
            Log.e(TAG, "Contours count: " + contours.size());
            Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

            Mat colorLabel = mRgba.submat(4, 50, 4, 50);
            colorLabel.setTo(mBlobColorRgba);

            Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70, 70 + mSpectrum.cols());
            mSpectrum.copyTo(spectrumLabel);
            String color = " "+Math.round(mBlobColorHsv.val[0])+"/"+Math.round(mBlobColorHsv.val[1])+"/"+Math.round(mBlobColorHsv.val[2])+"/"+Math.round(mBlobColorHsv.val[3])+"/"+mBlobColorHsv.val.length;
            //color = ""; // uncomment for debug purposes
            //command = 0;
        	Moments mo = new Moments();
        	int x = 0;
        	int y = 0;
            if (contours.size()>0) {
            	// set command by position of first contour
            	//command = 'f'; // implicit command
            	command = '-'; // implicit command
            	MatOfPoint contour = contours.get(0);
            	//Imgproc.fitEllipse(contour);
            	mo = Imgproc.moments(contour);
            	//MatOfPoint2f mc = Imgproc.
            	//( mo.get_m10()/mo.get_m00() , mo.get_m01()/mo.get_m00() );
                x = (int) (mo.get_m10() / mo.get_m00());
                y = (int) (mo.get_m01() / mo.get_m00());
                Core.circle(mRgba, new org.opencv.core.Point(x, y), 4, new Scalar(255,49,0,255));
                if (searchMode==0 || searchMode==2) {
                    if (x<p1) command = 'r';
                    if (x>p2) command = 'l';
                } else if (searchMode==1 || searchMode==3) {
                    if (x<p1) command = 'l';
                    if (x>p2) command = 'r';
                } 
            }
        	Core.putText(mRgba, "c: "+(char)command+" "+searchMode+"/"+x+"/"+y+color, new org.opencv.core.Point(4,(pos-30)), 1, siz, new Scalar(255,255,150), wi);
        	Core.putText(mRgba, "s: "+toRgba, new org.opencv.core.Point(4,pos), 1, siz, new Scalar(255,255,50), wi);

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
    		tmpText = tmpText.substring(0, tmpText.indexOf("\n")-1);
        	toRgba = tmpText;
        	tmpText = "";
    	}
    }

    static void setSearchMode(int sMode) {
    	searchMode = sMode;
    }

    private void sendCommand() {
        Timer timer = new Timer();
        timer.schedule(new TimerTask() {

          @Override
          public void run() {
            if (command!='-') {
    	    	out[0] = command;
             	MainActivity.mSerialService.write(out);
             	prevCommand = command;
             	command = '-';
            	tellCommand((char)out[0]);
            } else if (prevCommand!='-') {
            	// send 'f' once before nothing ('-')
    	    	out[0] = 'f';
             	MainActivity.mSerialService.write(out);
            	prevCommand = '-';
            	tellCommand((char)out[0]);
            }
          }
        },0,1000); //run every second
    }
    
    private void tellCommand(char command) {
    	String txtCommand = "";
    	if (command=='l') txtCommand = "left";
    	else if (command=='r') txtCommand = "right";
    	else if (command=='f') txtCommand = "straigt";
    	if (txtCommand!="") tts.speak(txtCommand, TextToSpeech.QUEUE_ADD, null);
    }
}
