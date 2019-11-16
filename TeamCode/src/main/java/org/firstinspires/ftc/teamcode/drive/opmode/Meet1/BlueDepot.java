package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous(group = "drive")
public class BlueDepot extends LinearOpMode {

    Servo frontGrabber;
    Servo rearGrabber;
    Servo foundationGrabber;

    boolean DONE = false;

    int skyStonePosition = -1;

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    // constants tuned with FtcDashboard
    final private static int fromBottom = 0;
    final private static int stoneHeight = 40;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera phoneCam;
    SkystonePipeline skystonePipeline;

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
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-41.32, 63.1, 0));

        frontGrabber = hardwareMap.servo.get("frontGrabber");
        rearGrabber = hardwareMap.servo.get("rearGrabber");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCam.openCameraDevice();

        skystonePipeline = new SkystonePipeline();
        phoneCam.setPipeline(skystonePipeline);
        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        ExecutorService networking = Executors.newSingleThreadExecutor();

        waitForStart();

        while (opModeIsActive() && !DONE) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(convertX(-67.168), convertY(62.690), 0), new ConstantInterpolator(Math.toRadians(0)))
                            .lineTo((new Vector2d(-67.168, 63.1)), new ConstantInterpolator(0))
                            .build()
            );

            networking.submit(submitImage);

            skyStonePosition = skystonePipeline.getPosition();

            telemetry.addData("skyStonePosition", skyStonePosition);
            telemetry.update();

            switch (skyStonePosition) {
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .back(1)
                                    .strafeRight(13)
//                                    .splineTo(new Pose2d(-70.24, 46.25, 0), new ConstantInterpolator(0))
                                    .build()
                    );
                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);
                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .forward(4)
                                    .strafeRight(13)
                                    .build()
                    );
                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);
                    break;
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(-67, 46.25, 0), new ConstantInterpolator(0))
                                    .strafeRight(13)
                                    .build()
                    );
                    frontGrabber.setPosition(0);
                    Thread.sleep(1000);
                    break;
            }

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(10, 55, 0), new ConstantInterpolator(0))
                            .strafeLeft(10)
                            .lineTo(new Vector2d(10, 55))
                            .build()
            );

            DONE = true;
        }
    }

    public class SkystonePipeline extends OpenCvPipeline {

        private int skystonePosition;

        final private static int rectHeight = frameHeight / 3;

        // defining stone detection zones
        Rect rectLeft = new Rect(fromBottom, 0, stoneHeight, rectHeight);
        Rect rectMiddle = new Rect(fromBottom, rectHeight, stoneHeight, rectHeight);
        Rect rectRight = new Rect(fromBottom, 2 * rectHeight, stoneHeight, rectHeight);

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

            return input;
        }

        protected int getPosition() {
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