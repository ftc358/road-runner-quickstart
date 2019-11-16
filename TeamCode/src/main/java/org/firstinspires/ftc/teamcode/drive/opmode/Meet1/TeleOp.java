package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp extends LinearOpMode {

    Servo frontGrabber;
    Servo rearGrabber;
    Servo foundationGrabber;

    double foundationGrabberState;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        frontGrabber = hardwareMap.servo.get("frontGrabber");
        rearGrabber = hardwareMap.servo.get("rearGrabber");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

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

            frontGrabber.setPosition(1 - gamepad1.left_trigger);
            rearGrabber.setPosition(gamepad1.right_trigger);
            foundationGrabber.setPosition(foundationGrabberState);
        }
    }
}