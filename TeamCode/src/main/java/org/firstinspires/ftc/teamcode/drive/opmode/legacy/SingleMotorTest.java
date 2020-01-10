package org.firstinspires.ftc.teamcode.drive.opmode.legacy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class SingleMotorTest extends LinearOpMode {

    DcMotor motor;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("power", motor.getPower());
            telemetry.addData("position",motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
