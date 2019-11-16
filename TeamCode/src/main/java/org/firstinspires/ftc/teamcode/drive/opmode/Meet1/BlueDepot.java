package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(group = "drive")
public class BlueDepot extends LinearOpMode {

    int skyStonePosition = -1;

    OpenCvCamera phoneCam;
    SkystonePipeline skystonePipeline;

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveREVOptimized drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-41.32, 63.1, 0));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = dashboard.getTelemetry();

        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        phoneCam.openCameraDevice();

        skystonePipeline = new SkystonePipeline();
        phoneCam.setPipeline(skystonePipeline);
        phoneCam.startStreaming(frameHeight, frameWidth, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
//                        .splineTo(new Pose2d(convertX(-67.168), convertY(62.690), 0), new ConstantInterpolator(Math.toRadians(0)))
                        .lineTo((new Vector2d(-67.168, 63.1)), new ConstantInterpolator(0))
                        .strafeRight(0.5)
                        .build()
        );

        skyStonePosition = skystonePipeline.getPosition();

        Thread.sleep(1000);

        telemetry.addData("Skystone position", skyStonePosition);
        telemetry.update();

        Thread.sleep(1000);
    }
}
