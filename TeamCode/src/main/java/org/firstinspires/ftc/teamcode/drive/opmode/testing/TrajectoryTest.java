package org.firstinspires.ftc.teamcode.drive.opmode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.util.AssetsTrajectoryLoader;

import java.io.IOException;

@Autonomous(group = "drive")
public class TrajectoryTest extends LinearOpMode {

//    Trajectory testTrajectory;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-48.00, 48.00, 0));

//        try {
//            testTrajectory = AssetsTrajectoryLoader.load("Test20191108-3");
//        } catch (IOException e) {
//            telemetry.addData("IOException", "Bad");
//        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(-67.168, 62.690, 0))
                        .build()
        );

//        drive.followTrajectorySync(testTrajectory);
    }
}
