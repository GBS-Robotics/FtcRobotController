package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArmServos", group = "Practice")
public class ServoArmTest extends LinearOpMode {

    static final int CYCLE_MS = 10;


    @Override
    public void runOpMode() {

        Servo armHandLeft = hardwareMap.get(Servo.class, "arm_hand_left");
        Servo armHandRight = hardwareMap.get(Servo.class, "arm_hand_right");
        DcMotor armBase = hardwareMap.get(DcMotor.class, "arm_base");
        double right = 0;
        double left = 0;

        waitForStart();

        while (opModeIsActive()) {

            //open claw
            if (gamepad2.left_bumper)
            {
                left = 1;
                right = 0;
            }

            //close claw
            if (gamepad2.right_bumper)
            {
                left = 0;
                right = 1;
            }

            armHandRight.setPosition(right);
            armHandLeft.setPosition(left);

            telemetry.addData(">", "Done");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", left, right);
            telemetry.update();

            idle();
        }

        // Signal done;
    }

}