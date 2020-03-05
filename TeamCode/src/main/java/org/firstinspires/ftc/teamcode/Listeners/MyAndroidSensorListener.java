package org.firstinspires.ftc.teamcode.Listeners;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyAndroidSensorListener implements SensorEventListener {
    private final SensorManager mSensorManager;
    private final Sensor mAccelerometer;
    private final Telemetry telemetry;


    public MyAndroidSensorListener(SensorManager sensorManager, Telemetry telemetry) {

        this.mSensorManager = sensorManager;
        this.telemetry = telemetry;

        this.mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);

    }

    public void onResume() {
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        float motionX = sensorEvent.values[0];
        float motionY = sensorEvent.values[1];
        float motionZ = sensorEvent.values[2];

        telemetry.addData("X Velocity", motionX);
        telemetry.addData("Y Velocity", motionY);
        telemetry.addData("Z Velocity", motionZ);
        telemetry.update();

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}
