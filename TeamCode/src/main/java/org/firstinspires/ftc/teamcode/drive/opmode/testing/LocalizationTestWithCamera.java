package org.firstinspires.ftc.teamcode.drive.opmode.testing;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.drive.opmode.legacy.InternalCameraExample;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class LocalizationTestWithCamera extends LinearOpMode {

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    public static int fromBottom = 20;
    public static int stoneHeight = 60;

    Servo foundationGrabber;
    Servo grabberWrist;
    Servo grabberHand;

    double foundationGrabberState;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera phoneCam;
    SamplePipeline pipeline;

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
    public void runOpMode() throws InterruptedException {

        grabberWrist = hardwareMap.servo.get("grabberWrist");
        grabberHand = hardwareMap.servo.get("grabberHand");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");

        grabberWrist.setPosition(0.7);
        grabberHand.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();
        pipeline = new SamplePipeline();
        phoneCam.setPipeline(pipeline);

        // comment out later
        ExecutorService networking = Executors.newSingleThreadExecutor();

        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (!isStopRequested()) {
            drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            drive.update();

            if (gamepad1.left_bumper) {
                foundationGrabberState = 0;
            } else if (gamepad1.right_bumper) {
                foundationGrabberState = 1;
            }

            foundationGrabber.setPosition(foundationGrabberState);

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("skystonePosition", pipeline.getPosition());
            telemetry.update();

            networking.submit(submitImage);
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