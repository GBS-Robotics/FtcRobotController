package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestArmServos", group = "Practice")
public class ServoArmTest extends LinearOpMode {

    static final double INCREMENT = 0.01;
    static final int CYCLE_MS = 50;
    static final double MAX_POS = 1.0;
    static final double MIN_POS = 0;

    public static double ARM_HAND_LEFT_POWER = 0;
    public static double ARM_HAND_RIGHT_POWER = 0;

    @Override
    public void runOpMode() {

        CRServo armHandLeft = hardwareMap.get(CRServo.class, "arm_hand_left");
        CRServo armHandRight = hardwareMap.get(CRServo.class, "arm_hand_right");
        CRServo armBase = hardwareMap.get(CRServo.class, "arm_base");

        waitForStart();

        while (opModeIsActive()) {

            if (ARM_HAND_LEFT_POWER > MAX_POS) {
                ARM_HAND_LEFT_POWER -= INCREMENT;
                armHandLeft.setPower(ARM_HAND_LEFT_POWER);
                armHandRight.setPower(-ARM_HAND_LEFT_POWER);
            } else if (ARM_HAND_LEFT_POWER < MIN_POS) {
                ARM_HAND_LEFT_POWER += INCREMENT;
                armHandLeft.setPower(ARM_HAND_LEFT_POWER);
                armHandRight.setPower(-ARM_HAND_LEFT_POWER);
            } else {
                ARM_HAND_LEFT_POWER = 0;
            }

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

}