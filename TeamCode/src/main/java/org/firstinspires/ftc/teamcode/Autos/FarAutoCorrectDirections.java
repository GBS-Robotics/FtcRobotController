package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class FarAutoCorrectDirections extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double max;

    @Override
    public void runOpMode() {
        // Base motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double max;

        double axial;
        double lateral;
        double yaw;

        waitForStart();
        runtime.reset();

        if (opModeIsActive())
        {
            setMotors(0, 0, 0);
            pause(20000/3);
            setMotors(0.5, 0, 0);
            pause(500/3);
            setMotors(0, 1, 0);
            pause(4250/3);
        }
    }

    private void setMotors(double axial, double lateral, double yaw)
    {
        leftFrontPower  = -axial - lateral + yaw;
        rightFrontPower = -axial + lateral - yaw;
        leftBackPower   = -axial + lateral + yaw;
        rightBackPower  = -axial - lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back left/right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();
    }

    private void pause(int time)
    {
        sleep(time);
        setMotors(0,0,0);
    }


    /*
    private turnDirection(boolean turnLeft, double waitSeconds) {
        if (turnLeft) {
            axial   = 0;
            lateral =  -dpadDriveSensitivity;
            yaw     =  0;
        }

        if (gamepad1.dpad_right) {
            axial   = 0;
            lateral =  dpadDriveSensitivity;
            yaw     =  0;
        }
    }
     */
}
