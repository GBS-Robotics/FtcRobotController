package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Calculators.CustomMathFunctions;

public class Claws {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares motors
    private Servo leftClaw = null;
    private Servo rightClaw = null;

    //Decleares variables
    private double leftPosition = 0f;
    private double rightPosition = 0f;

    //Declares constants
    private final double CLAW_SENSITIVITY = 0.01;

    Claws(String leftClawName, String rightClawName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        //Creates claws
        leftClaw = hardwareMap.get(Servo.class, leftClawName);
        rightClaw = hardwareMap.get(Servo.class, leftClawName);

        //Creates the telemetry
        telemetry = theTelemetry;
    }

    public void move(com. qualcomm. robotcore. hardware.Gamepad gamepad2) {
        //Gradual claw controls
        leftPosition += CLAW_SENSITIVITY * (gamepad2.left_trigger - gamepad2.right_trigger);
        rightPosition += CLAW_SENSITIVITY * (-gamepad2.left_trigger + gamepad2.right_trigger);

        //Open claw
        if (gamepad2.right_bumper) { //opens claws
            leftPosition = 1;
            rightPosition = 0;
        }

        //Close claw
        if (gamepad2.left_bumper) { //closes claw
            leftPosition = 0.42;
            rightPosition = 0.5;
        }

        //Send position to servos
        leftClaw.setPosition(leftPosition);
        rightClaw.setPosition(rightPosition);

        telemetry.addData("Claw Position left/right", "%4.2f, %4.2f", leftPosition, rightPosition);
        telemetry.addData("Claw Actual left/right", "%4.2f, %4.2f", leftClaw.getPosition(), rightClaw.getPosition());
    }
}