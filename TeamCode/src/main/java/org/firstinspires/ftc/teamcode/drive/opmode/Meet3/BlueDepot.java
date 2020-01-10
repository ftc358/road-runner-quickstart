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
public class BlueDepot extends LinearOpMode {

    // configurables

    // TODO: tune even more

    public static double stone0X = -21;
    public static double stone1X = -13;
    public static double stone2X = -17.5; //needs more testing

    public static double getStoneY = -26;

    public static double stone0Y = -26;
    public static double stone1Y = -26;
    public static double stone2Y = -26;

    public static double retractY = 10;

    public static double deliverX = -56;
    public static double secondDeliverYOffset = 4; // more tesing plz not next morning?

    public static double startX = 0;
    public static double startY = 0;

    public static double detectX = -17;
    public static double detectY = -12;

    public static double parkX = -38;

    // states

    boolean DONE = false;

    boolean X = true;

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

    Servo frontGrabber;
    Servo rearGrabber;
    Servo foundationGrabber;

    SampleMecanumDriveREVOptimized drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(startX, startY, 0));

        ConstantInterpolator zeroHeadingInterp = new ConstantInterpolator(0);

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
                            .strafeTo(new Vector2d(startX, detectY))
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo((new Vector2d(detectX, detectY)), zeroHeadingInterp)
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
                                    .lineTo(new Vector2d(stone0X, detectY), zeroHeadingInterp)
                                    .build()
                    );
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
                                    .lineTo(new Vector2d(stone1X, detectY), zeroHeadingInterp)
                                    .build()
                    );
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
                                    .lineTo(new Vector2d(stone2X, detectY), zeroHeadingInterp)
                                    .build()
                    );
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
                            .strafeLeft(retractY)
                            .build()
            );


            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY), zeroHeadingInterp)
                            .build()
            );

            delivery = 1;
            releaseStone();

            // get second stone; stoneX should be 24 greater than the first one

            switch (skyStonePosition) {
                case 0:
                    drive.followTrajectorySync(
                            drive.trajectoryBuilder()
                                    .lineTo(new Vector2d(stone0X + 24, getStoneY + retractY), zeroHeadingInterp)
                                    .build()
                    );
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
                                    .lineTo(new Vector2d(stone1X + 24, getStoneY + retractY), zeroHeadingInterp)
                                    .build()
                    );
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
                                    .lineTo(new Vector2d(stone2X + 24, getStoneY + retractY), zeroHeadingInterp)
                                    .build()
                    );
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
                            .strafeLeft(retractY + secondDeliverYOffset)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(deliverX, getStoneY + retractY + secondDeliverYOffset), zeroHeadingInterp)
                            .build()
            );

            delivery = 2;
            releaseStone();

            // park

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .lineTo(new Vector2d(parkX, getStoneY + retractY + secondDeliverYOffset), zeroHeadingInterp)
                            .build()
            );

            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .strafeRight(14)
                            .build()
            );

            DONE = true;
        }
    }

    public void getStone() throws InterruptedException {
        // TODO: implement
    }

    public void releaseStone() throws InterruptedException {
        // TODO: implement
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