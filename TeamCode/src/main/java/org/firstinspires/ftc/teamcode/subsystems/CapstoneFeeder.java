package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.CachingServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.Map;

public class CapstoneFeeder extends Subsystem {

    private Servo servo;

    private TelemetryData telemetryData;

    private double servoPosition;

    private class TelemetryData {
        public double servoPosition;
    }

    public CapstoneFeeder(HardwareMap map) {
        telemetryData = new TelemetryData();

        servo = new CachingServo(map.servo.get("capstoneFeeder"));

    }

    public void setServoPosition(double position) {
        servoPosition = position;
    }

    @Override
    public Map<String, Object> update(Canvas fieldOverlay) {
        telemetryData.servoPosition = servoPosition;
        servo.setPosition(servoPosition);
        return TelemetryUtil.objectToMap(telemetryData);
    }
}