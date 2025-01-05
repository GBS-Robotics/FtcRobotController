package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Calculators.CustomMathFunctions;

public class Lifter {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares motors
    private CRServo lifter = null;

    //Decleares variables
    private float lifterPower = 0f;

    //Declares constants
    private final float ARM_SENSITIVITY = 0.01f;

    Lifter(String deviceName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        lifter = hardwareMap.get(CRServo.class, deviceName);
        lifter.setDirection(CRServo.Direction.FORWARD);
        telemetry = theTelemetry;
    }

    public void move(com. qualcomm. robotcore. hardware.Gamepad gamepad1) {
        //lifter control
        lifterPower = gamepad1.right_trigger - gamepad1.left_trigger;

        //Makes idle power equal 1
        if (lifterPower == 0)
            lifterPower = 1;

        lifter.setPower(lifterPower);
        telemetry.addData("Lifter Power", "%4.2f", lifterPower);
    }
}
