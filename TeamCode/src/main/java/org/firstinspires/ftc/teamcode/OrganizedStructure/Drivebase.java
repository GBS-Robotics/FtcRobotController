package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Calculators.CustomMathFunctions;

public class Drivebase {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares motors
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //Declares variables
    private double axial;
    private double lateral;
    private double yaw;
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double max;

    //Declares constants
    private final float SLOW_SPEED = 0.25f;
    private final float REGULAR_SPEED = 1f;

    Drivebase(String leftFront, String rightFront, String leftBack, String rightBack, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        //Creates motor objects
        leftFrontDrive = hardwareMap.get(DcMotor.class, leftFront);
        leftBackDrive = hardwareMap.get(DcMotor.class, leftBack);
        rightFrontDrive = hardwareMap.get(DcMotor.class, rightFront);
        rightBackDrive = hardwareMap.get(DcMotor.class, rightBack);

        //Sets motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Sets motor zero power behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //creates the telemetry
        telemetry = theTelemetry;
    }

    public void move(com.qualcomm.robotcore.hardware.Gamepad gamepad1) {
        //Assigns desired directions directions
        axial = -gamepad1.left_stick_y * REGULAR_SPEED;
        lateral = gamepad1.left_stick_x * REGULAR_SPEED;
        yaw = gamepad1.right_stick_x * REGULAR_SPEED;

        //Slow backwards
        if (gamepad1.dpad_down) {
            axial = -SLOW_SPEED;
            lateral = 0;
            yaw = 0;
        }
        //Slow forwards
        if (gamepad1.dpad_up) {
            axial = SLOW_SPEED;
            lateral = 0;
            yaw = 0;
        }
        //Slow left
        if (gamepad1.dpad_left) {
            axial = 0;
            lateral = -SLOW_SPEED;
            yaw = 0;
        }
        //Slow right
        if (gamepad1.dpad_right) {
            axial = 0;
            lateral = SLOW_SPEED;
            yaw = 0;
        }

        //Calculates motor power based on desired directions
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        //Make sure power never goes above 1
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to motors
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        //Sends telemetry
        telemetry.addData("Front left/right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back left/right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }
}
