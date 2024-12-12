package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * @name Arm
 * @description Controls movement of arm so that it can be imported into DiveTeleOpCompact to reduce file size of the main file
 */
public class Arm {
    //variable declaration
    private DcMotorEx arm = null;
    private float power = 0f;
    private double DOWNSPEED = 0.5;
    private double UPSPEED = 1;
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    /**
     * @name Arm
     * @description Constructs arm with driver hub name direction and hardware map because it is more modular and less resource intensive.
     * @param deviceName
     * @param direction
     * @param hardwareMap
     * @param theTelemetry
     */
    Arm(String deviceName, DcMotorSimple.Direction direction, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        arm = hardwareMap.get(DcMotorEx.class, deviceName);
        arm.setDirection(direction);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = theTelemetry;
    }

    //sets the arm power based on user inputs
    public void move(float armPower) {
        //negative because up on joystick is a negative input, and down is positive, this compensates for that
        power = -1 * armPower;

        //changes power of arm if moving up or down to have faster up and slower down
        if (power < 0){
            power *= DOWNSPEED;
        }
        else if (power > 0) {
            power *= UPSPEED;
        }

        //sets power and also sends telemetry
        arm.setPower(power);
        telemetry.addData("Arm Power", "%4.2f", power);
    }
}
