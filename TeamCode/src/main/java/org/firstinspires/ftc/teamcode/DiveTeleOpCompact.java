package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DiveTeleOpCompact extends LinearOpMode {
    //creates timer for match
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Object instantiations
        Arm armBase = new Arm("arm_base", DcMotor.Direction.REVERSE, hardwareMap, telemetry);
        Slide linearSlide = new Slide("arm_base", DcMotor.Direction.REVERSE, hardwareMap, telemetry);

        //Confirms initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Prepares to start TeleOp
        waitForStart();
        runtime.reset();

        //Running TeleOp
        while (opModeIsActive()) {
            //Calls motor move functions
            armBase.move(gamepad2.right_stick_y);
            linearSlide.move(gamepad2.left_stick_y);
        }
    }
}
