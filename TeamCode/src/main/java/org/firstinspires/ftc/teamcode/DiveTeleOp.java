package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="DiveTeleOp")
public class DiveTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors, arm, and servos.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armBase = null;
    private Servo armHandLeft = null;
    private Servo armHandRight = null;

    @Override
    public void runOpMode() {

        // Base motors
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Arm servos and motors
        armHandLeft = hardwareMap.get(Servo.class, "arm_hand_left");
        armHandRight = hardwareMap.get(Servo.class, "arm_hand_right");
        armBase = hardwareMap.get(DcMotor.class, "arm_base");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double max;

        double axial;
        double lateral;
        double yaw;
        double armSpeed;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        double arm_left = 1;
        double arm_right = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;
            armSpeed = gamepad2.right_stick_y;

            leftFrontPower  = axial + yaw + lateral;
            rightFrontPower = axial - yaw - lateral;
            leftBackPower   = axial - yaw + lateral;
            rightBackPower  = axial + yaw - lateral;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
                armSpeed /= max;
            }

            if (gamepad2.right_bumper) {
                arm_left = 1;
                arm_right = 0;
            }

            if (gamepad2.left_bumper) {
                arm_left = 0.43;
                arm_right = 0.38;
            }

            if (gamepad2.right_trigger > 0.1) {
                armBase.setPower(gamepad2.right_trigger);
            }

            if (gamepad2.left_trigger > 0.1) {
                armBase.setPower(-0.5 * gamepad2.left_trigger);
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            armHandLeft.setPosition(arm_left);
            armHandRight.setPosition(arm_right);

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Hand left/right", "%4.2f, %4.2f", arm_left, arm_right);
            telemetry.addData("Servo", "%4.2f", armSpeed);
            telemetry.update();
        }
    }}
