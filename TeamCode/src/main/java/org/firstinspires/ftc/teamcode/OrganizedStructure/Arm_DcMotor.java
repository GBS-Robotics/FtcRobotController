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
    private int armPosition;

    //Declares constants
    private final float DOWNSPEED = 1;
    private final float UPSPEED = 1;
    private final float MAX_POWER = 1;
    private final float POSITION_SENSITIVITY = 1;
    private final float RAMP_AMOUNT = 0.01f;
    private final float INCRAMENTER = 0.0008f;

    /**
     * @name Arm
     * @description Constructs arm with driver hub name direction and hardware map because it is more modular and less resource intensive.
     * @param deviceName
     * @param hardwareMap
     * @param theTelemetry
     */
    Arm_DcMotor(String deviceName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        arm = hardwareMap.get(DcMotorEx.class, deviceName);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry = theTelemetry;
    }

    //sets the arm power based on user inputs
    public void move(float armPower) {
        //arm.setPower(0);
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
        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Sent Arm Power", "%4.2f", power);
    }

    public void move2(float armPower) {
        //arm.setPower(0);
        arm.setTargetPosition(arm.getCurrentPosition());
        if (armPower == 0) {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
        }
        else {
            //arm.setPower(0);
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

            //sets power
            arm.setPower(power);
        }

        //sends extra telemetry
        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Target Position", arm.getTargetPosition());
        telemetry.addData("Sent Arm Power", "%4.2f", power);
        telemetry.addData("Actual Arm Power", "%4.2f", arm.getPower());
    }

    public void move3(float armPower) {
        //arm.setPower(0);
        if (armPower == 0 && arm.getVelocity() != 0) {
            if (arm.getPower() < 0)
                power += RAMP_AMOUNT;
            else if (arm.getPower() > 0)
                power -= RAMP_AMOUNT;
        }
        else if (armPower == 0) {
            //arm.setTargetPosition(arm.getCurrentPosition());
            //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //arm.setPower(1);
            power = power;
        }
        else {
            this.move(armPower);
        }

        //sends telemetry
        telemetry.addData("Actual Arm Power", "%4.2f", arm.getPower());
    }

    public void move4(float joystick) {
        //power = (float) arm.getPower();
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        power += joystick * INCRAMENTER;

        //sets power and also sends telemetry
        arm.setPower(power);
        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Sent Arm Power", "%4.2f", power);
        telemetry.addData("Actual Arm Power", "%4.2f", arm.getPower());
    }

    public void move5(float armPower) {
        //arm.setPower(0);
        //arm.setTargetPosition(arm.getCurrentPosition());
        if (armPower == 0) {
            power = (float) arm.getPower();

            //sets power
            arm.setPower(power);
        }
        else {
            //arm.setPower(0);
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

            //sets power
            arm.setPower(power);
        }

        //sends extra telemetry
        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Target Position", arm.getTargetPosition());
        telemetry.addData("Sent Arm Power", "%4.2f", power);
        telemetry.addData("Actual Arm Power", "%4.2f", arm.getPower());
    }

    public void setPosition(float positionIncramenter) {
        /* Check to see if the arm is busy. If so, only update telemetry.
         This way, we don't interrupt the command while the firmware is
         still commanding the motor. */
        armPosition += POSITION_SENSITIVITY * positionIncramenter;
        arm.setTargetPosition(armPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(MAX_POWER);

        telemetry.addData("Current Position", arm.getCurrentPosition());
        telemetry.addData("Target Position", arm.getTargetPosition());
        telemetry.addData("Motor Power", arm.getPower());
    }
}
