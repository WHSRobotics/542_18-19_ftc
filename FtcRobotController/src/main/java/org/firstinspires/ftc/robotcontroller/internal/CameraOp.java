package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.ByteArrayOutputStream;
import java.io.IOException;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
@Autonomous(name = "TensorFlowTest", group = "tests")
public class CameraOp extends OpMode {
    private Camera camera;
    public CameraPreview preview;
    public Bitmap image;
    private int width;
    private int height;
    private YuvImage yuvImage = null;
    private int looped = 0;
    private String data;

    ImageClassifier imageClassifier;

    private Camera.PreviewCallback previewCallback = new Camera.PreviewCallback() {
        public void onPreviewFrame(byte[] data, Camera camera)
        {
            Camera.Parameters parameters = camera.getParameters();
            width = parameters.getPreviewSize().width;
            height = parameters.getPreviewSize().height;
            yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
            looped += 1;
        }
    };

    private void convertImage() {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();
        image = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length);
    }
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        camera = ((FtcRobotControllerActivity)hardwareMap.appContext).camera;
        camera.setPreviewCallback(previewCallback);

        Camera.Parameters parameters = camera.getParameters();
        data = parameters.flatten();

        ((FtcRobotControllerActivity) hardwareMap.appContext).initPreview(camera, this, previewCallback);

        try {
            imageClassifier = new ImageClassifier((Activity) hardwareMap.appContext);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    public int highestColor(int red, int green, int blue) {
        int[] color = {red,green,blue};
        int value = 0;
        for (int i = 1; i < 3; i++) {
            if (color[value] < color[i]) {
                value = i;
            }
        }
        return value;
    }

    @Override
    public void loop() {
        if (yuvImage != null) {
            convertImage();
            if(image != null) {
                String str = imageClassifier.classifyFrame(image);
                try {
                    telemetry.addData("Gold Position: ", (str == null | str.length() == 0) ? "null" : "not null");
                }
                catch (NullPointerException e){
                    e.printStackTrace();
                }
            }
        }
    }
}