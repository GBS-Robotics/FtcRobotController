package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slide {
    private DcMotorEx slide = null;
    private float power = 0f;
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    /**
     * @name Slide
     * @description Constructs slide with driver hub name, direction, and hardware map because it is more modular and less resource intensive.
     * @param deviceName
     * @param direction
     * @param hardwareMap
     * @param theTelemetry
     */
    Slide(String deviceName, DcMotorSimple.Direction direction, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        slide = hardwareMap.get(DcMotorEx.class, deviceName);
        slide.setDirection(direction);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry = theTelemetry;
    }

    //sets the arm power based on user inputs
    public void move(float slidePower) {
        //negative because up on the joystick is a negative input and down is positive, compensates for that
        power = -1 * slidePower;

        //sets power and also sends telemetry
        slide.setPower(power);
        telemetry.addData("Slide Power", "%4.2f", power);
    }

}
