package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "competition")
public class TeleOpOld extends LinearOpMode {

    Servo frontGrabber;
    Servo rearGrabber;
    Servo foundationGrabber;
    Servo capstoneFeeder;

//    double foundationGrabberState;
//    double capstoneFeederState;

//    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {

        frontGrabber = hardwareMap.servo.get("frontGrabber");
        frontGrabber.setDirection(Servo.Direction.REVERSE);
        rearGrabber = hardwareMap.servo.get("rearGrabber");
        foundationGrabber = hardwareMap.servo.get("foundationGrabber");
        capstoneFeeder = hardwareMap.servo.get("capstoneFeeder");

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            drive.setDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            drive.update();

            frontGrabber.setPosition(gamepad2.left_stick_y);
            rearGrabber.setPosition(-gamepad2.right_stick_y);
            foundationGrabber.setPosition(gamepad2.left_trigger);
            capstoneFeeder.setPosition(1 - gamepad2.right_trigger);
        }
    }
}