package org.firstinspires.ftc.teamcode.OrganizedStructure;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Calculators.CustomMathFunctions;

public class Arm_Servo {
    //Declares telemetry
    org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    //Declares motors
    private Servo arm = null;

    //Decleares variables
    private float position = 0f;

    //Declares constants
    private final float ARM_SENSITIVITY = 0.01f;

    Arm_Servo(String deviceName, com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, org.firstinspires.ftc.robotcore.external.Telemetry theTelemetry) {
        arm = hardwareMap.get(Servo.class, deviceName);
        arm.setDirection(Servo.Direction.REVERSE);
        telemetry = theTelemetry;
    }

    public void move(double joystick) {
        position += ARM_SENSITIVITY * joystick;
        position = CustomMathFunctions.bounds(position, 0, 1);

        arm.setPosition(position);
        telemetry.addData("Arm Position", "%4.2f", position);
    }
}
