package org.firstinspires.ftc.teamcode;

public class BasketTeleOpFunction {
    private double ARM_LENGTH = 23;
    //private double
    /**
     * @param heightCentimeters The height of the basket in centimeters
     * @name getAttachmentSteps
     * @description Returns motor instructions to raise the robot arm to the desired basket height.
     */
    public static double getPositionOfMotor(double heightCentimeters) {
        double maximumHeight = 69;

        return heightCentimeters / maximumHeight;
    }

    //public static (double, double) getSlideBasePositions(double )
}
