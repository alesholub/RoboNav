package cz.vmoste.robonav;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

public class Compass implements SensorEventListener {

	private SensorManager sensorManager;
	private Sensor gsensor;
	private Sensor msensor;
	private float[] mGravity = new float[3];
	private float[] mGeomagnetic = new float[3];
	private float iAzimuth = 999f;

	public Compass(Context context) {
		sensorManager = (SensorManager) context
				.getSystemService(Context.SENSOR_SERVICE);
		gsensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
		msensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
	}

	public void start() {
		sensorManager.registerListener(this, gsensor,
				SensorManager.SENSOR_DELAY_GAME);
		if (msensor!=null) sensorManager.registerListener(this, msensor,
				SensorManager.SENSOR_DELAY_GAME);
	}

	public void stop() {
		sensorManager.unregisterListener(this);
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		final float alpha = 0.98f;

		synchronized (this) {
			if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {

				mGravity[0] = alpha * mGravity[0] + (1 - alpha)
						* event.values[0];
				mGravity[1] = alpha * mGravity[1] + (1 - alpha)
						* event.values[1];
				mGravity[2] = alpha * mGravity[2] + (1 - alpha)
						* event.values[2];

			}

			if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
				
				mGeomagnetic[0] = alpha * mGeomagnetic[0] + (1 - alpha)
						* event.values[0];
				mGeomagnetic[1] = alpha * mGeomagnetic[1] + (1 - alpha)
						* event.values[1];
				mGeomagnetic[2] = alpha * mGeomagnetic[2] + (1 - alpha)
						* event.values[2];

			}

			if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
				float R[] = new float[9];
				float I[] = new float[9];
				boolean success = SensorManager.getRotationMatrix(R, I, mGravity,
						mGeomagnetic);
				if (success) {
					float orientation[] = new float[3];
					SensorManager.getOrientation(R, orientation);
					iAzimuth = (float) Math.toDegrees(orientation[0]); // orientation
					if (mGravity[0]>mGravity[1]) iAzimuth += 90f;
					iAzimuth = (iAzimuth + 360) % 360;
				} else {
					iAzimuth = 999;
				}
			}

		}
	}
	
	public int getAzimuth() {
		return (int)iAzimuth;
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}
}