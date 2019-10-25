package org.firstinspires.ftc.teamcode.drive.opmode.legacy;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Luke Intake Test", group = "LegacyTest")
public class LukeIntakeTest extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    ColorSensor color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    boolean captured = false;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
        color = hardwareMap.get(ColorSensor.class, "MRColor");
        color.enableLed(true);
    }

    @Override
    public void loop() {
        Color.RGBToHSV(color.red() * 8, color.green() * 8, color.blue() * 8, hsvValues);
        telemetry.addData("H", hsvValues[0]);
        telemetry.addData("S", hsvValues[1]);
        telemetry.addData("V", hsvValues[2]);
        telemetry.addData("Stop Condition", Math.abs(hsvValues[0] - 40));
        telemetry.addData("Captured", captured);
        telemetry.update();

        if ((Math.abs(hsvValues[0] - 40) < 10) && !captured) {
            motor1.setPower(0.3);
            motor2.setPower(-0.3);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                // whatever
            }
            captured = true;
        } else if (captured) {
            motor1.setPower(0);
            motor2.setPower(0);
        } else {
            motor1.setPower(-gamepad1.left_stick_y);
            motor2.setPower(gamepad1.left_stick_y);
        }

        if (gamepad1.a) {
            captured = false;
        }
    }
}
