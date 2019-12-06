package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Disabled
@Config
@Autonomous(group = "competition")
public class RedDepotOld extends LinearOpMode {

    // configurables

    //TODO: tune
    public static double stone0X = -66;
    //TODO: tune
    public static double stone1X = -58;
    //TODO: tune
    public static double stone2X = -63;

    //TODO: tune
    public static double getStoneY = -34;

    public static double stone0Y = -34;
    public static double stone1Y = -34;
    public static double stone2Y = -34;

    //TODO: tune
    public static double retractY = -12;

    public static double secondYOffset = -2;

    //TODO: tune--see if farthest stone needs separate
    public static double deliverX = 10;

    public static double startX = -41.32;
    public static double startY = -63.1;

    public static double detectX = -70;
    public static double detectY = -52;

    // states

    boolean DONE = false;

    int skyStonePosition = 2;

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
        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(180)));

        ConstantInterpolator backHeadingInterp = new ConstantInterpolator(Math.toRadians(180));

        frontGrabber = hardwareMap.servo.get("frontGrabber");
        rearGrabber = hardwareMap.servo.get("rearGrabber");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCam.openCameraDevice();

        skystonePipeline = new SkystonePipeline();
        phoneCam.setPipeline(skystonePipeline);
        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive() && !DONE) {
            // to detection
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(-67.168, 62.690, 0), new ConstantInterpolator(Math.toRadians(0)))
                            .strafeTo(new Vector2d(startX, detectY))
                            .lineTo((new Vector2d(detectX, detectY)), backHeadingInterp)
                            .build()
            );

            skyStonePosition = skystonePipeline.getPosition();

            telemetry.addData("skyStonePosition", skyStonePosition);
            telemetry.update();

            //get first stone

            switch (skyStonePosition) {
                // the x coordinates should be 8 inches apart
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X, detectY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone0X, stone0Y))
                                    .build()
                    );
                    getStoneY = stone0Y;
                    getStone();
                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone1X, detectY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone1X, stone1Y))
                                    .build()
                    );
                    getStoneY = stone1Y;
                    getStone();
                    break;
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone2X, detectY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone2X, stone2Y))
                                    .build()
                    );
                    getStoneY = stone2Y;
                    getStone();
                    break;
            }

            // deliver first stone

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(retractY)
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY), backHeadingInterp)
                            .build()
            );

            releaseStone();

            // get second stone; stoneX should be 24 greater than the first one

            switch (skyStonePosition) {
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X + 24, getStoneY + retractY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone0X + 24, getStoneY + secondYOffset))
                                    .build()
                    );

                    getStone();
                    Thread.sleep(1000);

                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone1X + 24, getStoneY + retractY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone1X + 24, getStoneY + secondYOffset))
                                    .build()
                    );

                    getStone();
                    Thread.sleep(1000);

                    break;
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone2X + 24 - 0.5, getStoneY + retractY), backHeadingInterp)
                                    .strafeTo(new Vector2d(stone2X + 24 - 0.5, getStoneY + secondYOffset))
                                    .build()
                    );

                    getStone();
                    Thread.sleep(1000);

                    break;
            }

            // deliver second stone

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(retractY)
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY + secondYOffset), backHeadingInterp)
                            .build()
            );

            releaseStone();

            // park

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(-8, getStoneY + secondYOffset + retractY), backHeadingInterp)
                            .build()
            );

            DONE = true;
        }
    }

    public void getStone() throws InterruptedException {
        switch (skyStonePosition) {
            case 2:
                frontGrabber.setPosition(0);
                Thread.sleep(1000);
                break;
            case 1:
                frontGrabber.setPosition(0);
                Thread.sleep(1000);
                break;
            case 0:
                rearGrabber.setPosition(1.0);
                Thread.sleep(1000);
                break;
        }
    }

    public void releaseStone() throws InterruptedException {
        switch (skyStonePosition) {
            case 2:
                frontGrabber.setPosition(1.0);
                Thread.sleep(1000);
                break;
            case 1:
                frontGrabber.setPosition(1.0);
                Thread.sleep(1000);
                break;
            case 0:
                rearGrabber.setPosition(0);
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