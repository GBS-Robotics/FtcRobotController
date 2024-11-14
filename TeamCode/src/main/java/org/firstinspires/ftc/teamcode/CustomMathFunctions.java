package org.firstinspires.ftc.teamcode;

public class CustomMathFunctions {

    public static float degreesToMotorTicks(float degrees) {
        return degrees / 1.25f;
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }
}
