package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "DiveTeleOp (don't use)")
public class DiveTeleOpCompact extends LinearOpMode {
    //creates timer for match
    private final ElapsedTime runtime = new ElapsedTime();

    //Claw Code wouldn't work :(
    private Servo armHandLeft = null;
    private Servo armHandRight = null;

    @Override
    public void runOpMode() {
        //Object instantiations
        Drivebase drivebase = new Drivebase("left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", hardwareMap, telemetry);
        Arm_DcMotor armBase = new Arm_DcMotor("arm_base", hardwareMap, telemetry);
        Claws claws = new Claws("arm_hand_left", "arm_hand_right", hardwareMap, telemetry);
        Slide linearSlide1 = new Slide("slide", hardwareMap, telemetry);
        Slide linearSlide2 = new Slide("slide2", hardwareMap, telemetry);
        Lifter lifter = new Lifter("left_lifter", hardwareMap, telemetry);

        //Claw Code wouldn't work :(
        armHandLeft = hardwareMap.get(Servo.class, "arm_hand_left");
        armHandRight = hardwareMap.get(Servo.class, "arm_hand_right");

        //Claw Code wouldn't work :(
        double claw_left = 1;
        double claw_right = 0;

        //Confirms initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Prepares to start TeleOp
        waitForStart();
        runtime.reset();

        //Running TeleOp
        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addLine("---------------------------------");

            //Calls motor move functions
            drivebase.move(gamepad1);
            armBase.move2(gamepad2.right_stick_y); //I tried many versions to make it hold in place, but this is the only one that seems to work.
            //claws.move(gamepad2);
            linearSlide1.move(gamepad2.left_stick_y);
            linearSlide2.move(gamepad2.left_stick_y);
            lifter.move(gamepad1);

            //Claw Code wouldn't work :(
            //Claw open and close controls
            claw_left += 0.01 * (-gamepad2.left_trigger + gamepad2.right_trigger);
            claw_right += 0.01 * (gamepad2.left_trigger - gamepad2.right_trigger);

            //Open claw
            if (gamepad2.right_bumper) { //opens claws
                claw_left = 1;
                claw_right = 0;
            }

            //Close claw
            if (gamepad2.left_bumper) { //closes claw
                claw_left = 0.42;
                claw_right = 0.5;
            }

            //Send position to servos
            armHandLeft.setPosition(claw_left);
            armHandRight.setPosition(claw_right);

            //Adds right stick telemetry and updates
            telemetry.addData("Right Stick", "%4.2f", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
