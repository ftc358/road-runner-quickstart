package org.firstinspires.ftc.teamcode.drive.opmode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import static org.firstinspires.ftc.teamcode.drive.opmode.testing.FieldVelocityTeleOpTest.CONTROL_MODE.ABSOLUTE;
import static org.firstinspires.ftc.teamcode.drive.opmode.testing.FieldVelocityTeleOpTest.CONTROL_MODE.RELATIVE;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class FieldVelocityTeleOpTest extends LinearOpMode {

    public enum CONTROL_MODE {
        ABSOLUTE,
        RELATIVE
    }

    private CONTROL_MODE mode = CONTROL_MODE.RELATIVE;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {

            Pose2d currentPose = drive.getPoseEstimate();

            if (gamepad1.a) {
                mode = ABSOLUTE;
                drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), 0));
            } else if (gamepad1.b) {
                mode = RELATIVE;
            }

            if (mode == RELATIVE) {
                drive.setDrivePower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                ));
            } else if (mode == ABSOLUTE) {
                drive.setDrivePower(Kinematics.fieldToRobotVelocity(currentPose, new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )));
            }


            drive.update();

            telemetry.addData("x", currentPose.getX());
            telemetry.addData("y", currentPose.getY());
            telemetry.addData("heading", currentPose.getHeading());
            telemetry.addData("mode", mode);
            telemetry.update();
        }
    }
}