package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "DiveTeleOpCompactTest")
public class DiveTeleOpCompact extends LinearOpMode {
    //creates timer for match
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Object instantiations
        Drivebase drivebase = new Drivebase("left_front_drive", "right_front_drive", "left_back_drive", "right_back_drive", hardwareMap, telemetry);
        Arm_Servo armBase = new Arm_Servo("arm_base", hardwareMap, telemetry);
        Claws claws =new Claws("arm_hand_left", "arm_hand_right", hardwareMap, telemetry);
        Slide linearSlide = new Slide("slide", hardwareMap, telemetry);
        Lifter lifter = new Lifter("left_lifter", hardwareMap, telemetry);

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
            armBase.move(gamepad2.right_stick_y);
            claws.move(gamepad2);
            linearSlide.move(gamepad2.left_stick_y);
            lifter.move(gamepad1);

            telemetry.update();
        }
    }
}
