package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slide {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares Motors
    private DcMotorEx slide = null;

    //Declares variables
    private float power = 0f;

    /**
     * @name Slide
     * @description Constructs slide with driver hub name, direction, and hardware map because it is more modular and less resource intensive.
     * @param deviceName
     * @param hardwareMap
     * @param theTelemetry
     */
    Slide(String deviceName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        slide = hardwareMap.get(DcMotorEx.class, deviceName);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        telemetry = theTelemetry;
    }

    //sets the arm power based on user inputs
    public void move(float joystick) {
        //negative because up on the joystick is a negative input and down is positive, compensates for that
        power = -1 * joystick;

        //sets power and also sends telemetry
        slide.setPower(power);
        telemetry.addData("Slide Power", "%4.2f", power);
    }

}
