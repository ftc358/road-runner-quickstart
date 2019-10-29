package org.firstinspires.ftc.teamcode.drive.opmode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motors Test", group = "LegacyTest")
public class MotorsTest extends OpMode {
    DcMotor motor1, motor2, motor3, motor4;

    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2 = hardwareMap.dcMotor.get("motor2");
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3 = hardwareMap.dcMotor.get("motor3");
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4 = hardwareMap.dcMotor.get("motor4");
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.left_stick_y);
        motor3.setPower(gamepad1.left_stick_y);
        motor4.setPower(gamepad1.left_stick_y);
        telemetry.addData("1:", motor1.getCurrentPosition());
        telemetry.addData("2:", motor2.getCurrentPosition());
        telemetry.addData("3:", motor3.getCurrentPosition());
        telemetry.addData("4:", motor4.getCurrentPosition());
        telemetry.update();
    }
}
