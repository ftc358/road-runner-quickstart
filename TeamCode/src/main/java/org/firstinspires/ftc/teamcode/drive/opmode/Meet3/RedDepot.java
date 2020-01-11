package org.firstinspires.ftc.teamcode.drive.opmode.Meet3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
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

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

@Config
@Autonomous(group = "competition")
public class RedDepot extends LinearOpMode {

    // configurables

    public static double stone0X = -4;
    public static double stone1X = -12;
    public static double stone2X = -19;

    public static double getStoneY = 27;

    public static double stone0Y = 30;
    public static double stone1Y = 30;
    public static double stone2Y = 30;

    public static double retractY = -15;

    public static double deliverX = 56;
    public static double secondDeliverYOffset = -4;

    public static double startX = 0;
    public static double startY = 0;

    public static double detectX = -21;
    public static double detectY = 12;

    public static double parkX = 38;

    // states

    boolean DONE = false;

    int delivery = 0;

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

    Servo foundationGrabber;
    Servo grabberWrist;
    Servo grabberHand;

    SampleMecanumDriveREVOptimized drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startX, startY, Math.toRadians(180)));

        ConstantInterpolator backHeadingInterp = new ConstantInterpolator(Math.toRadians(180));

        grabberWrist = hardwareMap.servo.get("grabberWrist");
        grabberHand = hardwareMap.servo.get("grabberHand");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");

        grabberWrist.setPosition(0.8);
        grabberHand.setPosition(0);

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
                            .strafeTo(new Vector2d(startX, detectY))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo((new Vector2d(detectX, detectY)), backHeadingInterp)
                            .build()
            );

            skyStonePosition = skystonePipeline.getPosition();

            telemetry.addData("skyStonePosition", skyStonePosition);
            telemetry.update();

            //get first stone
            switch (skyStonePosition) {
                // the x coordinates should be 8 inches apart
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X, detectY), backHeadingInterp)
//                                    .lineTo(new Vector2d(stone0X, detectY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
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
//                                    .lineTo(new Vector2d(stone1X, detectY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(stone1X, stone1Y))
                                    .build()
                    );
                    getStoneY = stone1Y;
                    getStone();
                    break;
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone2X, detectY), backHeadingInterp)
//                                    .lineTo(new Vector2d(stone2X, detectY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
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
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY), backHeadingInterp)
                            .build()
            );

            delivery = 1;
            releaseStone();

            // get second stone; stoneX should be 24 greater than the first one

            switch (skyStonePosition) {
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X + 24, getStoneY + retractY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(stone0X + 24, getStoneY))
                                    .build()
                    );

                    getStone();

                    break;
                case 1:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone1X + 24, getStoneY + retractY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(stone1X + 24, getStoneY))
                                    .build()
                    );

                    getStone();

                    break;
                case 2:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone2X + 24, getStoneY + retractY), backHeadingInterp)
                                    .build()
                    );
                    lowerArm();
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .strafeTo(new Vector2d(stone2X + 24, getStoneY))
                                    .build()
                    );

                    getStone();

                    break;
            }

            // deliver second stone

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(retractY + secondDeliverYOffset)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY + secondDeliverYOffset), backHeadingInterp)
                            .build()
            );

            delivery = 2;
            releaseStone();

            // park

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(parkX, getStoneY + retractY + secondDeliverYOffset), backHeadingInterp)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(16)
                            .build()
            );

            DONE = true;
        }
    }

    public void lowerArm() throws InterruptedException {
        grabberHand.setPosition(1);
        grabberWrist.setPosition(0);
        Thread.sleep(1000);
    }

    public void getStone() throws InterruptedException {
        grabberHand.setPosition(0);
        Thread.sleep(1500);
        grabberWrist.setPosition(0.8);
        Thread.sleep(1000);
    }

    public void releaseStone() throws InterruptedException {
        grabberWrist.setPosition(0.5);
        grabberHand.setPosition(1);
        Thread.sleep(500);
        grabberWrist.setPosition(0.8);
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