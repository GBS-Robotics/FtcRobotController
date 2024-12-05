package org.firstinspires.ftc.teamcode;

public class BasketTeleOpFunction {
    /**
     * @name getAttachmentSteps
     * @description Returns motor instructions to raise the robot arm to the desired basket height.
     * @param heightCentimeters The height of the basket in centimeters
     *
     */
    public static void getPositionOfMotor(double heightCentimeters) {
        double maximumHeight = 69;
        double position = heightCentimeters / maximumHeight;

//        return position;
    }
}
