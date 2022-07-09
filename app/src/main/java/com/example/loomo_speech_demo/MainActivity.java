package com.example.loomo_speech_demo;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Gravity;
import android.view.Surface;
import android.view.TextureView;
import android.widget.CompoundButton;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.Switch;
import android.widget.TextView;
import com.segway.robot.sdk.vision.DTS;

import java.io.File;

import com.segway.robot.algo.Pose2D;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.locomotion.head.Head;
import com.segway.robot.sdk.perception.sensor.Sensor;
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.locomotion.sbv.Base;
import com.segway.robot.sdk.vision.stream.StreamType;
import com.segway.robot.sdk.voice.Recognizer;
import com.segway.robot.sdk.voice.VoiceException;
import com.segway.robot.sdk.voice.grammar.GrammarConstraint;
import com.segway.robot.sdk.voice.grammar.Slot;
import com.segway.robot.sdk.voice.recognition.RecognitionListener;
import com.segway.robot.sdk.voice.recognition.RecognitionResult;
import com.segway.robot.sdk.voice.recognition.WakeupListener;
import com.segway.robot.sdk.voice.recognition.WakeupResult;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

public class MainActivity extends AppCompatActivity implements CompoundButton.OnCheckedChangeListener {
    Recognizer mRecognizer;
    Base mBase;
    ServiceBinder.BindStateListener mRecognizerBindStateListener;
    ServiceBinder.BindStateListener mBaseBindStateListener;
    ServiceBinder.BindStateListener mVisionBindStateListener;
    ServiceBinder.BindStateListener mSensorBindStateListener;
    ServiceBinder.BindStateListener mHeadBindStateListener;
    GrammarConstraint movementGrammar;
    WakeupListener mWakeupListener;
    RecognitionListener mRecognitionListener;
    PreviewPresenter mPreviewPresenter;
    Vision mVision;
    Sensor mSensor;
    Head mHead;
    DTS mDTS;
    ImageView mColorImageView;
    ImageView mDepthImageView;
    ImageView mFishImageView;
    TextureView mTextureView;
    private TransferPresenter mTransferPresenter;
    private Switch mTransferSwitch;

    private IImageState mIImageState = new IImageState() {

        Runnable mRunnable;

        @Override
        public void updateImage(int type, final Bitmap bitmap) {
            switch (type) {
                case StreamType.COLOR:
                    mRunnable = new Runnable() {
                        @Override
                        public void run() {
                            mColorImageView.setImageBitmap(bitmap);
                        }
                    };
                    break;
                case StreamType.DEPTH:
                    mRunnable = new Runnable() {
                        @Override
                        public void run() {
                            mDepthImageView.setImageBitmap(bitmap);
                        }
                    };
                    break;
                case StreamType.FISH_EYE:
                    mRunnable = new Runnable() {
                        @Override
                        public void run() {
                            mFishImageView.setImageBitmap(bitmap);
                        }
                    };
                    break;
            }

            if (mRunnable != null) {
                runOnUiThread(mRunnable);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        mBase = Base.getInstance();
        mSensor = Sensor.getInstance();
        mRecognizer = Recognizer.getInstance();
        mVision = Vision.getInstance();
        mHead = Head.getInstance();
        initListeners();

        mTransferSwitch = (Switch) findViewById(R.id.transfer);
        mColorImageView = (ImageView) findViewById(R.id.colorImage);
        mDepthImageView = (ImageView) findViewById(R.id.depthImage);
        mFishImageView = (ImageView) findViewById(R.id.fishImage);
        mTextureView = (TextureView) findViewById(R.id.trackSurface);

        mTransferSwitch.setOnCheckedChangeListener(this);
        mTransferPresenter = new TransferPresenter(mVision, mBase, mSensor, mIImageState, mTextureView);

        TextView textViewIp = (TextView) findViewById(R.id.textView_ip);
        textViewIp.setText(getDeviceIp());

        mRecognizer.bindService(getApplicationContext(), mRecognizerBindStateListener);
        mBase.bindService(getApplicationContext(), mBaseBindStateListener);
        mVision.bindService(this, mVisionBindStateListener);
        mSensor.bindService(this, mSensorBindStateListener);
        mHead.bindService(this, mHeadBindStateListener);


        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        deleteContent(path);
//        this.finish();
    }

    void deleteContent(File fileOrDirectory) {
        int length = 1;
        int progress = 0;
        if (fileOrDirectory.isDirectory())
            length = fileOrDirectory.listFiles().length + 1;
            for (File child : fileOrDirectory.listFiles()) {
                Log.d("REMOVE ", "delete" + child.getName() + ", progress: " + (float) progress / length);
                child.delete();
            }
    }

    @Override
    protected void onResume() {
        super.onResume();
        mTransferSwitch.setChecked(false);
    }

    @Override
    protected void onStop() {
        super.onStop();
        mDTS.stop();
        mRecognizer.unbindService();
        mBase.unbindService();
        mVision.unbindService();
        mTransferPresenter.stop();
        finish();
    }

    private void initListeners() {

        mRecognizerBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                try {
                    initGrammar();
                    // Wake-up and Recognition Mode
                    mRecognizer.startWakeupAndRecognition(mWakeupListener, mRecognitionListener);
                    // Recognition only mode
//                    mRecognizer.startRecognitionMode(mRecognitionListener);
                } catch (VoiceException e) {}


            }

            @Override
            public void onUnbind(String reason) {

            }
        };

        mWakeupListener = new WakeupListener() {
            @Override
            public void onStandby() {

            }

            @Override
            public void onWakeupResult(WakeupResult wakeupResult) {

            }

            @Override
            public void onWakeupError(String error) {

            }
        };

        mRecognitionListener = new RecognitionListener() {
            @Override
            public void onRecognitionStart() {

            }

            @Override
            public boolean onRecognitionResult(RecognitionResult recognitionResult) {
                String result = recognitionResult.getRecognitionResult();
                baseOriginReset();
                if (result.contains("rotate")) {
                    if (result.contains("left")) {
                        // rotate left
                        mBase.addCheckPoint(0f,0f, (float) Math.PI/2);
                    } else if (result.contains("right")) {
                        // rotate right
                        mBase.addCheckPoint(0f, 0f, (float) (-1*Math.PI)/2);
                    }

                } else if (result.contains("move") || result.contains("go") || result.contains("turn")) {
                    if (result.contains("forward")) {
                        mBase.addCheckPoint(1f, 0f);
                    } else if (result.contains("backward")) {
                        mBase.addCheckPoint(-1f, 0f);
                    } else if ( result.contains("left")) {
                        mBase.addCheckPoint(0f, 1f);
                    } else if (result.contains("right")) {
                        mBase.addCheckPoint(0f, -1f);
                    }

                }
                return true;
            }

            @Override
            public boolean onRecognitionError(String error) {
                return true;
            }
        };



        mBaseBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                mTransferPresenter.baseBinded = true;
                mBase.setControlMode(Base.CONTROL_MODE_RAW);
            }

            @Override
            public void onUnbind(String reason) {

            }
        };

        mVisionBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                mDTS = Vision.getInstance().getDTS();
                mDTS.setVideoSource(DTS.VideoSource.CAMERA);
                SurfaceTexture mSurfaceTexture = mTextureView.getSurfaceTexture();
//                mSurfaceTexture.setDefaultBufferSize(3264, 1836);
                mSurfaceTexture.setDefaultBufferSize(306, 544);

//                mSurfaceTexture.setOnFrameAvailableListener(new SurfaceTexture.OnFrameAvailableListener(){
//                    @Override
//                    public void onFrameAvailable(SurfaceTexture surfaceTexture) {
//                        mTransferPresenter.updateTexture();
//                    }
//                });
                mTextureView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
                    @Override
                    public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture, int i, int i1) {

                    }

                    @Override
                    public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture, int i, int i1) {

                    }

                    @Override
                    public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
                        return false;
                    }

                    @Override
                    public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {
                        mTransferPresenter.updateTexture();
                    }
                });
                Surface mSurface = new Surface(mSurfaceTexture);
                mDTS.setPreviewDisplay(mSurface);
                mDTS.start();
                mTransferSwitch.setEnabled(true);
                mTransferSwitch.setChecked(true);
            }

            @Override
            public void onUnbind(String reason) {
            }
        };

        mSensorBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                mTransferPresenter.sensorBinded = true;

            }

            @Override
            public void onUnbind(String reason) {

            }
        };

        mHeadBindStateListener = new ServiceBinder.BindStateListener() {
            @Override
            public void onBind() {
                Pose2D newOriginPoint = mBase.getOdometryPose(-1);
                mBase.setOriginalPoint(newOriginPoint);
                mHead.setMode(Head.MODE_SMOOTH_TACKING);
                mHead.setWorldPitch(0f);
                mHead.setWorldYaw((float) 0f);
            }

            @Override
            public void onUnbind(String reason) {

            }
        };

    }

    private void initGrammar() throws VoiceException {
        Slot firstSlot = new Slot("movement");
        firstSlot.addWord("move");
        firstSlot.addWord("go");
        firstSlot.addWord("turn");
        firstSlot.addWord("rotate");

        Slot secondSlot = new Slot("direction");
        secondSlot.addWord("forward");
        secondSlot.addWord("backward");
        secondSlot.addWord("left");
        secondSlot.addWord("right");

        List<Slot> movementSlotList = new LinkedList<>();
        movementSlotList.add(firstSlot);
        movementSlotList.add(secondSlot);

        movementGrammar = new GrammarConstraint("movements", movementSlotList);
        mRecognizer.addGrammarConstraint(movementGrammar);


    }

    private void baseOriginReset() {
        mBase.setControlMode(Base.CONTROL_MODE_NAVIGATION);
        //mBase.clearCheckPointsAndStop();
        mBase.cleanOriginalPoint();
        Pose2D newOriginPoint = mBase.getOdometryPose(-1);
        mBase.setOriginalPoint(newOriginPoint);
    }

    private String getDeviceIp() {
        WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        if (!wifiManager.isWifiEnabled()) {
            wifiManager.setWifiEnabled(true);
        }
        WifiInfo wifiInfo = wifiManager.getConnectionInfo();
        int ipAddress = wifiInfo.getIpAddress();
        String ip = (ipAddress & 0xFF) + "." +
                ((ipAddress >> 8) & 0xFF) + "." +
                ((ipAddress >> 16) & 0xFF) + "." +
                (ipAddress >> 24 & 0xFF);
        return ip;
    }

    @Override
    public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
        switch (buttonView.getId()) {
            case R.id.transfer:
                if (isChecked) {
                    mTransferPresenter.start();
                } else {
                    mTransferPresenter.stop();
                }
                break;
        }
    }
}
