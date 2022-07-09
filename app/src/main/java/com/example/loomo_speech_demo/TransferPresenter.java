package com.example.loomo_speech_demo;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.ColorMatrixColorFilter;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.util.Log;
import android.view.TextureView;

import com.segway.robot.algo.Pose2D;
import com.segway.robot.algo.minicontroller.CheckPoint;
import com.segway.robot.algo.minicontroller.CheckPointStateListener;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.perception.sensor.SensorData;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.frame.Frame;
import com.segway.robot.sdk.vision.imu.IMUDataCallback;
import com.segway.robot.sdk.vision.stream.StreamInfo;
import com.segway.robot.sdk.vision.stream.StreamType;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.Random;
import java.util.SimpleTimeZone;
import java.util.TimeZone;

import static java.lang.Math.log;
import static java.lang.Math.min;

/**
 * @author jacob
 * @date 5/7/18
 */

public class TransferPresenter {

    private static final String TAG = "TransferPresenter";
    private final Base mBase;

    private final Vision mVision;
    private final Sensor mSensor;
    private final IImageState mIImageState;
    private final TextureView mTextureView;
    public boolean baseBinded = false;
    public boolean sensorBinded = false;
    public boolean recoverMode = false;
    public int sleep = -5;

    private final Paint mPaint = new Paint();
    private final Canvas mCanvas = new Canvas();
    private Bitmap fishBitmapToSave;

    private StreamInfo mColorInfo;
    private StreamInfo mDepthInfo;
    private StreamInfo mFishInfo;
    private final Random rd = new Random();
    private Bitmap mColorBitmap;
    private Bitmap mDepthBitmap;
    private Bitmap mFishBitmap;
    private float mAngularVelocity = 10f;
    private float mLinearVelocity = 10f;
    private float mInfraredDistanceLeft = 100;
    private float mInfraredDistanceRight = 100;
    private float mUltrasonicDistance = 0;
    private float x, y, mTheta;
    private float linear = 0.f;
    private float angular = 0.f;
    private int controlStep = 451;
    private boolean autoExposure = false;

    public TransferPresenter(Vision mVision, Base mBase, Sensor mSensor, IImageState mIImageState, TextureView mTextureView) {
        this.mVision = mVision;
        this.mBase = mBase;
        this.mSensor = mSensor;
        this.mIImageState = mIImageState;
        this.mTextureView = mTextureView;
    }

    public synchronized void start() {
        Log.d(TAG, "start() called");
        StreamInfo[] streamInfos = mVision.getActivatedStreamInfo();
        for (StreamInfo info : streamInfos) {
            switch (info.getStreamType()) {
                case StreamType.COLOR:
                    mColorInfo = info;
                    mVision.startListenFrame(StreamType.COLOR, mFrameListener);
                    break;
                case StreamType.DEPTH:
                    mDepthInfo = info;
                    mVision.startListenFrame(StreamType.DEPTH, mFrameListener);
                    break;
                case StreamType.FISH_EYE:
                    mFishInfo = info;
                    mVision.startListenFrame(StreamType.FISH_EYE, mFrameListener);
                    break;
            }
        }
        mColorBitmap = Bitmap.createBitmap(mColorInfo.getWidth(), mColorInfo.getHeight(), Bitmap.Config.ARGB_8888);
        mDepthBitmap = Bitmap.createBitmap(mDepthInfo.getWidth(), mDepthInfo.getHeight(), Bitmap.Config.RGB_565);
        mFishBitmap = Bitmap.createBitmap(mFishInfo.getWidth(), mFishInfo.getHeight(), Bitmap.Config.ALPHA_8);

        final Handler handler = new Handler();
        Runnable policy = new Runnable() {
            @Override
            public void run() {
                handler.postDelayed(this, 250);
                if (baseBinded && sensorBinded){
                    controlStep += 1;
                    getSensorData();
                    float obstacleDistance = min(min(mUltrasonicDistance, mInfraredDistanceLeft), mInfraredDistanceRight);
                    controlPolicy(obstacleDistance);
                    writeSensor();
                }
            }
        };
        policy.run();

    }

    private void writeSensor() {
        String logString =
                "controlStep: " + controlStep +
                ", mInfraredDistanceLeft: " + mInfraredDistanceLeft +
                ", mInfraredDistanceRight: " + mInfraredDistanceRight +
                ", mUltrasonicDistance: " + mUltrasonicDistance +
                ", x: " +  x +
                ", y: " + y +
                ", mTheta: " + mTheta +
                ", mLinearVelocity: " + mLinearVelocity +
                ", mAngularVelocity: " + mAngularVelocity +
                ", linear: " + linear +
                ", angular: " + angular +
                ", recoverMode: " + recoverMode +
                ", sleep: " + sleep;
        Log.d(TAG, logString);
        try {
            File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
            String filename = "sensor.txt";
            File file = new File(path, "/" + filename);
            FileOutputStream fOut = new FileOutputStream(file, true);
            OutputStreamWriter osw = new OutputStreamWriter(fOut);
            osw.write(logString + "\n");
            osw.flush();
            osw.close();
            fOut.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    private void getSensorData() {
        SensorData mUltrasonicData = mSensor.querySensorData(Arrays.asList(Sensor.ULTRASONIC_BODY)).get(0);
//                    SensorData mWheelSpeed = mSensor.querySensorData(Arrays.asList(Sensor.WHEEL_SPEED)).get(0);
//                    float mWheelSpeedL = mWheelSpeed.getFloatData()[0];
//                    float mWheelSpeedR = mWheelSpeed.getFloatData()[1];
        SensorData mInfraredData = mSensor.querySensorData(Arrays.asList(Sensor.INFRARED_BODY)).get(0);
        mInfraredDistanceLeft = mInfraredData.getIntData()[0];
        mInfraredDistanceRight = mInfraredData.getIntData()[1];
        mUltrasonicDistance = mUltrasonicData.getIntData()[0];
        SensorData mPose2DData = mSensor.querySensorData(Arrays.asList(Sensor.POSE_2D)).get(0);
        Pose2D pose2D = mSensor.sensorDataToPose2D(mPose2DData);
        x = pose2D.getX();
        y = pose2D.getY();
        mTheta = pose2D.getTheta();
        mLinearVelocity = pose2D.getLinearVelocity();
        mAngularVelocity = pose2D.getAngularVelocity();
    }

    public synchronized void stop() {
        Log.d(TAG, "stop() called");
        mVision.stopListenFrame(StreamType.COLOR);
        mVision.stopListenFrame(StreamType.DEPTH);
        mVision.stopListenFrame(StreamType.FISH_EYE);
    }

    private Calendar getTime(){
        // get the supported ids for GMT-08:00 (Pacific Standard Time)
        String[] ids = TimeZone.getAvailableIDs(-8 * 60 * 60 * 1000);
        // create a Pacific Standard Time time zone
        SimpleTimeZone pdt = new SimpleTimeZone(-8 * 60 * 60 * 1000, ids[0]);
        // set up rules for Daylight Saving Time
        pdt.setStartRule(Calendar.APRIL, 1, Calendar.SUNDAY, 2 * 60 * 60 * 1000);
        pdt.setEndRule(Calendar.OCTOBER, -1, Calendar.SUNDAY, 2 * 60 * 60 * 1000);
        // create a GregorianCalendar with the Pacific Daylight time zone
        // and the current date and time
        Calendar calendar = new GregorianCalendar(pdt);
        Date trialTime = new Date();
        calendar.setTime(trialTime);
        return calendar;
    }

    private String getTimeName(){
        Calendar calendar = this.getTime();
        return calendar.get(Calendar.YEAR) + "_" +
                (1 + calendar.get(Calendar.MONTH)) + "_" +
                calendar.get(Calendar.DATE) + "_" +
                calendar.get(Calendar.HOUR_OF_DAY) + "-" +
                calendar.get(Calendar.MINUTE) + "-" +
                calendar.get(Calendar.SECOND) + "-" +
                calendar.get(Calendar.MILLISECOND);
    }

    private void recover() {
        recoverMode = false;
        mBase.setControlMode(Base.CONTROL_MODE_NAVIGATION);
        mBase.clearCheckPointsAndStop();
        mBase.cleanOriginalPoint();
        Pose2D newOriginPoint = mBase.getOdometryPose(-1);
        mBase.setOriginalPoint(newOriginPoint);
        mBase.setOnCheckPointArrivedListener(new CheckPointStateListener() {
            @Override
            public void onCheckPointArrived(CheckPoint checkPoint, Pose2D realPose, boolean isLast) {
                mBase.setControlMode(Base.CONTROL_MODE_RAW);
                recoverMode = true;
            }

            @Override
            public void onCheckPointMiss(CheckPoint checkPoint, Pose2D realPose, boolean isLast, int reason) {

            }
        });
    }

    private void convertAlpha8(Bitmap src) {
        fishBitmapToSave = Bitmap.createBitmap(mFishInfo.getWidth(), mFishInfo.getHeight(), Bitmap.Config.ARGB_8888);
        mCanvas.setBitmap(fishBitmapToSave);
        ColorMatrixColorFilter mColorFilter = new ColorMatrixColorFilter(new float[]{
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 1, 0
        });
        mPaint.setColorFilter(mColorFilter);
        mCanvas.drawBitmap(src, 0, 0, mPaint);
    }

    /**
     * FrameListener instance for get raw image data form vision service
     */
    Vision.FrameListener mFrameListener = new Vision.FrameListener() {

        @Override
        public void onNewFrame(int streamType, Frame frame) {
            if (!autoExposure) {
                mVision.enableFishEyeAutoExposure(true);
                autoExposure = true;
            }

            switch (streamType) {
                case StreamType.COLOR:
                    // draw color image to bitmap and display
                    mColorBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                    mIImageState.updateImage(StreamType.COLOR, mColorBitmap);
                    writeImages(streamType);
                    break;
                case StreamType.DEPTH:
                    // draw depth image to bitmap and display
                    mDepthBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                    mIImageState.updateImage(StreamType.DEPTH, mDepthBitmap);
                    writeImages(streamType);
                    break;
                case StreamType.FISH_EYE:
                    // draw depth image to bitmap and display
                    mFishBitmap.copyPixelsFromBuffer(frame.getByteBuffer());
                    mIImageState.updateImage(StreamType.FISH_EYE, mFishBitmap);
                    writeImages(streamType);
                    break;
            }

        }
    };

    private void writeImages(int streamType) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = streamType + "_" + getTimeName() + "_" + controlStep + ".png";
        File file = new File(path, "/" + filename);

        try (FileOutputStream out = new FileOutputStream(file)) {
            switch (streamType) {
                case StreamType.COLOR:
                    mColorBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    out.flush();
                    break;
                case StreamType.DEPTH:
                    mDepthBitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                    out.flush();
                    break;
                case StreamType.FISH_EYE:
                    convertAlpha8(mFishBitmap);
                    fishBitmapToSave.compress(Bitmap.CompressFormat.PNG, 100, out);
                    out.flush();
                    break;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void controlPolicy(float obstacleDistance) {
        if (!recoverMode) {
            if ((obstacleDistance < 500) || ((sleep > -1) && (!(mAngularVelocity > 0.01)) && (!(mLinearVelocity > 0.01
            )))) {
                // switch to recover mode
                recoverMode = true;
                sleep = 0;
//                        if (mInfraredDistanceLeft < mInfraredDistanceRight) {
//                            mBase.setAngularVelocity(-0.2f);
//                        } else {
//                            mBase.setAngularVelocity(0.2f);
//                        }
                angular = 0.f;
                linear = 0.f;
            } else {
                sleep = min(sleep+1, 0);
//                        sleep = 0;
                angular = rd.nextFloat() * 0.5f - 0.25f;
                linear = rd.nextFloat() * 0.3f + 0.1f;
            }
            mBase.setAngularVelocity(angular);
            mBase.setLinearVelocity(linear);
        } else {
            Log.d(TAG, "recover mode");
//                    if (mInfraredDistanceLeft < mInfraredDistanceRight) {
//                        mBase.setAngularVelocity(-0.2f);
//                    } else {
//                        mBase.setAngularVelocity(0.2f);
//                    }
            sleep += 1;
            if (sleep < 10) {
                linear = 0.f;
                angular = 0.f;
            } else if (sleep < 30) {
                linear = -0.2f;
                angular = 0.f;
            } else if (sleep <=40){
                linear = 0.f;
                angular = 0.342f;
            } else {
                linear = 0.f;
                angular = 0.f;
                recoverMode = false;
                sleep = -10;
            }
            mBase.setAngularVelocity(angular);
            mBase.setLinearVelocity(linear);
        }
    }

    void updateTexture(){
        Bitmap bitmap = mTextureView.getBitmap(306, 544);
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "HD_" + getTimeName() + "_" + controlStep + ".png";
        File file = new File(path, "/" + filename);
        try (FileOutputStream out = new FileOutputStream(file)) {
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            out.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
