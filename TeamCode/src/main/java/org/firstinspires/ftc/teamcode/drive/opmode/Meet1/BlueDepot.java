package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(group = "drive")
public class BlueDepot extends LinearOpMode {

    // configurables

    public static double getStoneY = 50;

    public static double stone0X = -69;
    public static double stone1X = -61;
    public static double stone2X = -67.168;

    public static double startX = -41.32;
    public static double startY = 63.1;

    // states

    boolean DONE = false;

    int skyStonePosition = -1;

    // consts

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    final private static int fromBottom = 20;
    final private static int stoneHeight = 60;

    // objects

    FtcDashboard dashboard = FtcDashboard.getInstance();

    OpenCvCamera phoneCam;
    SkystonePipeline skystonePipeline;

    Servo frontGrabber;
    Servo rearGrabber;
    Servo foundationGrabber;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startX, startY, 0));

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
            // to detection
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(convertX(-67.168), convertY(62.690), 0), new ConstantInterpolator(Math.toRadians(0)))
//                            .strafeTo(new Vector2d(-67.168, 62.690))
                            .lineTo((new Vector2d(-67.168, startY)), new ConstantInterpolator(0))
                            .build()
            );

            skyStonePosition = skystonePipeline.getPosition();

            telemetry.addData("skyStonePosition", skyStonePosition);
            telemetry.update();

            //get first stone

            switch (skyStonePosition) {
                //TODO: the x coordinates should be 8 inches apart; test tomorrow
                //TODO: get all the stone X coordinates & getStoneY
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
//                                    .splineTo(new Pose2d(-70.24, 46.25, 0), new ConstantInterpolator(0))
                                    .lineTo(new Vector2d(stone0X, startY))
//                                    .back(1)
                                    .strafeTo(new Vector2d(stone0X, getStoneY))
//                                    .strafeRight(13)
                                    .build()
                    );
                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);
                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone1X, startY))
//                                    .forward(4)
                                    .strafeTo(new Vector2d(stone1X, getStoneY))
//                                    .strafeRight(13)
                                    .build()
                    );
                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);
                    break;
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(-67, 46.25, 0), new ConstantInterpolator(0))
                                    .lineTo(new Vector2d(stone2X, startY))
                                    .strafeTo(new Vector2d(stone2X, getStoneY))
//                                    .strafeRight(13)
                                    .build()
                    );
                    frontGrabber.setPosition(0);
                    Thread.sleep(1000);
                    break;
            }

            // deliver first stone

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(10, 55, 0), new ConstantInterpolator(0))
                            .strafeLeft(5)
                            .lineTo(new Vector2d(10, getStoneY + 5))
                            .build()
            );

            releaseStone();

            // get second stone

            switch (skyStonePosition) {
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X + 24, getStoneY + 5))
                                    .strafeTo(new Vector2d(stone0X + 24, getStoneY))
                                    .build()
                    );

                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);

                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone1X + 24, getStoneY + 5))
                                    .strafeTo(new Vector2d(stone1X + 24, getStoneY))
                                    .build()
                    );

                    rearGrabber.setPosition(1.0);
                    Thread.sleep(1000);

                    break;
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone2X + 24, getStoneY + 5))
                                    .strafeTo(new Vector2d(stone2X + 24, getStoneY))
                                    .build()
                    );

                    frontGrabber.setPosition(0);
                    Thread.sleep(1000);

                    break;
            }

            // deliver second stone

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                            .splineTo(new Pose2d(10, 55, 0), new ConstantInterpolator(0))
                            .strafeLeft(5)
                            .lineTo(new Vector2d(10, getStoneY + 5))
                            .build()
            );

            releaseStone();

            // park

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                    .lineTo(new Vector2d(0, getStoneY + 5))
                    .build()
            );

            DONE = true;
        }
    }

    public void releaseStone() throws InterruptedException {
        switch (skyStonePosition) {
            case 0:
                rearGrabber.setPosition(0);
                Thread.sleep(1000);
                break;
            case 1:
                rearGrabber.setPosition(0);
                Thread.sleep(1000);
                break;
            case 2:
                frontGrabber.setPosition(1.0);
                Thread.sleep(1000);
                break;
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