package org.firstinspires.ftc.teamcode.drive.opmode.legacy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Ryan Intake Test", group = "Legacy Test")
public class RyanIntakeTest extends OpMode {
    DcMotor motorL;
    DcMotor motorR;

    public void init() {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
    }

    public void loop() {
        motorL.setPower(gamepad1.left_stick_y);
        motorR.setPower(-gamepad1.left_stick_y);
    }
}
