package org.firstinspires.ftc.teamcode.drive.opmode.testing;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Config
@TeleOp
public class CameraTest extends LinearOpMode {

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    public static int fromBottom = 5;
    public static int stoneHeight = 40;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera phoneCam;
    SamplePipeline samplePipeline;

    // comment out later
    Bitmap bmp = null;

    // comment out later
    Runnable submitImage = new Runnable() {
        @Override
        public void run() {
            dashboard.sendImage(bmp);
        }
    };

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        samplePipeline = new SamplePipeline();
        phoneCam.setPipeline(samplePipeline);

        // comment out later
        ExecutorService networking = Executors.newSingleThreadExecutor();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            // comment out later
            networking.submit(submitImage);

            telemetry.addData("skystonePosition", samplePipeline.getPosition());
            telemetry.update();
        }
    }

    protected class SamplePipeline extends OpenCvPipeline {

        private int skystonePosition;
        Scalar green = new Scalar(0, 0, 255);

        // defining stone detection zones
        Rect rectLeft = new Rect(fromBottom, 0, stoneHeight, frameHeight / 3);
        Rect rectMiddle = new Rect(fromBottom, frameHeight / 3, stoneHeight, frameHeight / 3);
        Rect rectRight = new Rect(fromBottom, 2 * frameHeight / 3, stoneHeight, frameHeight / 3);

        @Override
        public Mat processFrame(Mat input) {

            Mat left = new Mat(input, rectLeft);
            Mat middle = new Mat(input, rectMiddle);
            Mat right = new Mat(input, rectRight);

            double leftValue = getValue(Core.mean(left));
            double middleValue = getValue(Core.mean(middle));
            double rightValue = getValue(Core.mean(right));

            // make an arrayList with the values of the 3 blocks for easier comparison
            ArrayList<Double> values = new ArrayList<>();
            values.add(leftValue);
            values.add(middleValue);
            values.add(rightValue);

            skystonePosition = values.indexOf(Collections.min(values));

            // drawing things on the screen so that things can be seen on screen
            Imgproc.rectangle(input, rectLeft, green, 2);
            Imgproc.rectangle(input, rectMiddle, green, 2);
            Imgproc.rectangle(input, rectRight, green, 2);

            // convert the image to a bitmap to be sent to the dashboard
            try {
                bmp = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(input, bmp);
            } catch (CvException e) {
                Log.d("Bitmap Creation Failure", e.getMessage());
            }

            return input;
        }

        private int getPosition() {
            return skystonePosition;
        }

        private double getValue(Scalar scalar) {
            double max = 0;

            for (int i = 0; i < 3; i++) {
                if (max < scalar.val[i]) {
                    max = scalar.val[i];
                }
            }

            return max;
        }

    }
}
