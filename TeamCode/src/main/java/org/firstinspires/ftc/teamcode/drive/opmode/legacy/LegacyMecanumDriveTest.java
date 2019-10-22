package org.firstinspires.ftc.teamcode.drive.opmode.legacy;

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

    protected DcMotor leftFront;
    protected DcMotor leftRear;
    protected DcMotor rightFront;
    protected DcMotor rightRear;

    double SCALE = 1;

    public void runOpMode() throws InterruptedException {

        waitForStart();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.dcMotor.get("leftRear");
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.dcMotor.get("rightRear");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            leftFront.setPower(-(POWER * flPower / maxPower) / SCALE);                                                 //
            leftRear.setPower(-(POWER * blPower / maxPower) / SCALE);                                                 //
            rightFront.setPower(-(POWER * frPower / maxPower) / SCALE);                                                 //
            rightRear.setPower(-(POWER * brPower / maxPower) / SCALE);                                                 //
            //Drive code :)///////////////////////////////////////////////////////////////////////////////////////

            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("leftRear", leftRear.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("rightRear", rightRear.getCurrentPosition());
            telemetry.update();
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
