package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

@TeleOp
public class Demo extends OpMode {

    private Robot robot;
    private StickyGamepad stickyGamepad;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad = new StickyGamepad(gamepad1);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        stickyGamepad.update();

        robot.foundationGrabber.setServoPosition(gamepad1.left_trigger);
        robot.capstoneFeeder.setServoPosition(0.95 - gamepad1.right_trigger);
    }
}
