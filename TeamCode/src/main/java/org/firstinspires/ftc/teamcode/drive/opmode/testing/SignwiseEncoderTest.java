package org.firstinspires.ftc.teamcode.drive.opmode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SignwiseEncoderTest extends OpMode {
    DcMotor fakeMotor1, fakeMotor2, fakeMotor3;

    public void init() {
        fakeMotor1 = hardwareMap.dcMotor.get("fakeMotor1");
        fakeMotor2 = hardwareMap.dcMotor.get("fakeMotor2");
        fakeMotor3 = hardwareMap.dcMotor.get("fakeMotor3");
    }

    public void loop() {
        telemetry.addData("1 (front) current position:", fakeMotor1.getCurrentPosition());
        telemetry.addData("2 (left) current position:", fakeMotor2.getCurrentPosition());
        telemetry.addData("3 (right) current position:", fakeMotor3.getCurrentPosition());
        telemetry.update();
    }
}
