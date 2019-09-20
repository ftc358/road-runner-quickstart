package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

@TeleOp
public class LegacyMecanumDriveTest extends LinearOpMode {

    protected DcMotor fL;
    protected DcMotor bL;
    protected DcMotor fR;
    protected DcMotor bR;

    double SCALE = 1;

    public void runOpMode() throws InterruptedException {

        waitForStart();

        fL = hardwareMap.dcMotor.get("leftFront");
        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL = hardwareMap.dcMotor.get("leftRear");
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR = hardwareMap.dcMotor.get("rightFront");
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR = hardwareMap.dcMotor.get("rightRear");
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeIsActive()) {

            //Drive code :) //////////////////////////////////////////////////////////////////////////////////////
            //Defining drive, strafe, and rotation power.                                                       //
            double drive = -gamepad1.left_stick_y;                                                               //
            double strafe = -gamepad1.left_stick_x;                                                              //
            double rotate = gamepad1.right_stick_x;                                                             //
            //
            //Defining the motor power distribution.                                                            //
            double flPower = drive - strafe + rotate;                                                           //
            double blPower = drive + strafe + rotate;                                                           //
            double frPower = drive + strafe - rotate;                                                           //
            double brPower = drive - strafe - rotate;                                                           //
            //
            double joyStick = Range.clip(max(magnitudeLeftStick(gamepad1), abs(rotate)), -1, 1);     //
            double POWER = -1 * joyStick * abs(joyStick);                                                       //
//            telemetry.addData("POWER: ", POWER);                                                        //
            double maxPower = findMax(abs(flPower), abs(blPower), abs(frPower), abs(brPower));                  //
            // greatest value of all motor powers                                                               //
//            telemetry.addData("maxPower: ", maxPower);                                                  //
            //
            //Sets the power for all the drive motors.                                                          //
            fL.setPower(-(POWER * flPower / maxPower) / SCALE);                                                 //
            bL.setPower(-(POWER * blPower / maxPower) / SCALE);                                                 //
            fR.setPower(-(POWER * frPower / maxPower) / SCALE);                                                 //
            bR.setPower(-(POWER * brPower / maxPower) / SCALE);                                                 //
            //Drive code :)///////////////////////////////////////////////////////////////////////////////////////
        }
    }

    //This function finds the magnitude of the left stick of a gamepad.
    private Double magnitudeLeftStick(Gamepad gamepad) {
        return sqrt(pow(gamepad.left_stick_x, 2) + pow(gamepad.left_stick_y, 2));
    }

    //This function finds the max value given 4 values.
    private Double findMax(Double d1, Double d2, Double d3, Double d4) {
        return max(max(d1, d2), max(d3, d4));
    }
}
