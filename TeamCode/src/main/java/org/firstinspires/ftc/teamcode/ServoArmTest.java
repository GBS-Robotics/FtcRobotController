package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArmServos", group = "Practice")
public class ServoArmTest extends LinearOpMode {

    public static double arm_left = 0;
    public static double arm_right = 0;

    @Override
    public void runOpMode() {

        Servo armHandLeft = hardwareMap.get(Servo.class, "arm_hand_left");
        Servo armHandRight = hardwareMap.get(Servo.class, "arm_hand_right");
        DcMotor armBase = hardwareMap.get(DcMotor.class, "arm_base");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.right_bumper) {
                arm_left = 0;
                arm_right = 1;
            }

            if (gamepad2.left_bumper) {
                arm_left = 1;
                arm_right = 0;
            }

            if (gamepad2.right_trigger > 0.1) {
                armBase.setPower(gamepad2.right_trigger);
            }

            if (gamepad2.left_trigger > 0.1) {
                armBase.setPower(-gamepad2.left_trigger);
            }

            armHandLeft.setPosition(arm_left);
            armHandRight.setPosition(arm_right);

            telemetry.addData("Left/Right", "%4.2f, %4.2f", arm_left, arm_right);
            telemetry.update();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

}