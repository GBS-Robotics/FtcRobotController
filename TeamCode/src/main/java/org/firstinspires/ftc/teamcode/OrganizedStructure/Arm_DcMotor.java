package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * @name Arm
 * @description Controls movement of arm so that it can be imported into DiveTeleOpCompact to reduce file size of the main file
 */
public class Arm_DcMotor {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares motors
    private DcMotorEx arm = null;

    //Declares variables
    private float power = 0f;

    //Declares constants
    private double DOWNSPEED = 0.5;
    private double UPSPEED = 1;


    /**
     * @name Arm
     * @description Constructs arm with driver hub name direction and hardware map because it is more modular and less resource intensive.
     * @param deviceName
     * @param hardwareMap
     * @param theTelemetry
     */
    Arm_DcMotor(String deviceName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        arm = hardwareMap.get(DcMotorEx.class, deviceName);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = theTelemetry;
    }

    //sets the arm power based on user inputs
    public void move(float armPower) {
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void setPosition(int position) {
        /* Check to see if the arm is busy. If so, only update telemetry.
         This way, we don't interrupt the command while the firmware is
         still commanding the motor. */
        arm.setTargetPosition(position);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);

        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Target Position", arm.getTargetPosition());
        telemetry.update();
    }
}
