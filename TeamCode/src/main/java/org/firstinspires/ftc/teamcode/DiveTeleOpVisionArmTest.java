package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.os.Build;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "DiveTeleOpVisionArmTest")
public class DiveTeleOpVisionArmTest extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private final VectorF aprilTagDistance = new VectorF(0.0f, 0.0f, 0.0f);
    Quaternion q = new Quaternion();
    EulerAngle angles = new EulerAngle(q);
    RotateRobot relativeField = new RotateRobot(0, aprilTagDistance);

    // Declare OpMode members for each of the 4 motors, arm, and servos.
    private final ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime current_time = new ElapsedTime();
    private ElapsedTime previous_time = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx armBase = null;
    private Servo armHandLeft = null;
    private Servo armHandRight = null;
    private DcMotor slide = null;

    @Override
    public void runOpMode() {

        // Base motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Arm servos and motors
        armHandLeft = hardwareMap.get(Servo.class, "arm_hand_left");
        armHandRight = hardwareMap.get(Servo.class, "arm_hand_right");
        armBase = hardwareMap.get(DcMotorEx.class, "arm_base");
        slide = hardwareMap.get(DcMotor.class, "slide");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armBase.setDirection(DcMotor.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setTargetPosition(0);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */

        double max;

        double axial;
        double lateral;
        double yaw;

        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        double slideSpeed;
        double armPosition = 0.0;
        double armPower = 0.0;
        //double armMaxVelocity = 0;

        boolean isControllingArm;

        int current_error;
        int previous_error = 0;
        double p = 0, i = 0, d = 0;
        double k_p = 0.1;
        double k_i = 0;
        double k_d = 0;

        double claw_left = 1;
        double claw_right = 0;

        final double ARM_DOWN_SENSITIVITY = 0.02;
        final double ARM_UP_SENSITIVITY = 1.0;
        final double FALLING_SENSITIVITY = 0.0001;
        final double DRIVE_BASE_SENSITIVITY = 1;
        final double DPAD_SENSITIVITY = 0.25;
        final double CLAW_SENSITIVITY = 0.01;
        final double ARM_VELOCITY_SENSITIVITY = 0.5;
        final double ARM_POSITION_SENSITIVITY = 0.1;

        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //armPosition = 0.0;
            //Driving controls
            axial = -gamepad1.left_stick_y * DRIVE_BASE_SENSITIVITY;
            lateral = gamepad1.left_stick_x * DRIVE_BASE_SENSITIVITY;
            yaw = gamepad1.right_stick_x * DRIVE_BASE_SENSITIVITY;

            //Linear slide control
            slideSpeed = gamepad2.left_stick_y;

            //Claw open and close controls
            claw_left += CLAW_SENSITIVITY * (gamepad2.left_trigger - gamepad2.right_trigger);
            claw_right += CLAW_SENSITIVITY * (-gamepad2.left_trigger + gamepad2.right_trigger);

            //Sets that we're not controlling the arm
            isControllingArm = false;

            //Move arm full speed
//            if (gamepad2.x) {
//                armPosition = gamepad2.right_stick_y;
//                isControllingArm = true;
//            }
//            else if (gamepad2.right_stick_y < 0) {
//                //Move arm up at partial speed
//                armPosition = ARM_UP_SENSITIVITY * gamepad2.right_stick_y;
//                isControllingArm = true;
//            }
//            else if (gamepad2.right_stick_y > 0){
//                //Move arm down at partial speed
//                armPosition = ARM_DOWN_SENSITIVITY * gamepad2.right_stick_y;
//                isControllingArm = true;
//            }

            //Stops the arm from falling
            /*
            if(!isControllingArm) {
                //armPosition = -FALLING_SENSITIVITY * armBase.getVelocity();
                armPosition = 0.00001;
            }
             */

            //p decrease
            if (gamepad2.dpad_down) {
                k_p -= 0.0001;
            }

            //p increase
            if (gamepad2.dpad_up) {
                k_p += 0.0001;
            }

            //d decrease
            if (gamepad2.dpad_left) {
                k_d -= 0.0001;
            }

            //d increase
            if (gamepad2.dpad_right) {
                k_d += 0.0001;
            }

            k_p = CustomMathFunctions.bounds((float) k_p, 0, 1);
            k_d = CustomMathFunctions.bounds((float) k_d, 0, 1);

            //moves arm
            armPosition = gamepad2.right_stick_y;

            //PID for arm
            current_time = runtime;
            current_error = (int) CustomMathFunctions.degreesToMotorTicks((float) armPosition) - armBase.getCurrentPosition();

            p = k_p * current_error;

            i += k_i * (current_error * (current_time.milliseconds() - previous_time.milliseconds()));

            /*
            if (i > max_i)
            i = max_i
            else if (i <-max_i)
            i = -max_i
             */

            d = k_d * (current_error - previous_error) / (current_time.milliseconds() - previous_time.milliseconds());

            armPower = (p + i + d);

            previous_error = current_error;
            previous_time = current_time;


            //keeps arm in bounds
            //armPosition = CustomMathFunctions.bounds((float) armPosition, -360, 360);

            /*
            if (gamepad2.dpad_down) {
              armMaxVelocity -= ARM_VELOCITY_SENSITIVITY;
            }

            //Slow forwards
            if (gamepad2.dpad_up) {
                armMaxVelocity += ARM_VELOCITY_SENSITIVITY;
            }
             */

            //Open claw
            if (gamepad2.right_bumper) { //opens claws
                claw_left = 1;
                claw_right = 0;
            }

            //Close claw
            if (gamepad2.left_bumper) { //closes claw
                claw_left = 0.5;
                claw_right = 0.55;
            }

            //Slow backwards
            if (gamepad1.dpad_down) {
                axial = -DPAD_SENSITIVITY;
                lateral = 0;
                yaw = 0;
            }

            //Slow forwards
            if (gamepad1.dpad_up) {
                axial = DPAD_SENSITIVITY;
                lateral = 0;
                yaw = 0;
            }

            //Slow left
            if (gamepad1.dpad_left) {
                axial = 0;
                lateral = -DPAD_SENSITIVITY;
                yaw = 0;
            }

            //Slow right
            if (gamepad1.dpad_right) {
                axial = 0;
                lateral = DPAD_SENSITIVITY;
                yaw = 0;
            }

            //Assign the power variables.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            //Make sure power never goes above 1
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
                armPower /= max;
            }

            // Send calculated power to motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armBase.setPower(armPosition);
            slide.setPower(slideSpeed);

            //Send position to servos
            armHandLeft.setPosition(claw_left);
            armHandRight.setPosition(claw_right);

            //Send telemetry
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back left/right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Hand left/right", "%4.2f, %4.2f", claw_left, claw_right);
            telemetry.addData("Kp - Kd", "%4.2f, %4.2f", k_p, k_d);
            telemetry.addData("Error", current_error);
            telemetry.addData("Right Stick", "%4.2f", gamepad2.right_stick_y);
            telemetry.addData("Arm Power", "%4.2f", armPower);
            telemetry.addData("Target Position", "%4.2f", armPosition);
            telemetry.addData("Arm Position", -armBase.getCurrentPosition());
            //telemetry.addData("Arm Velocity", "%4.2f", armMaxVelocity);
            //telemetry.addData("velocity", armBase.getVelocity());
            //telemetry.addData("position", CustomMathFunctions.motorTicksToDegrees(armBase.getCurrentPosition()));
            //telemetry.addData("is at target", !armBase.isBusy());
            telemetry.addData("Controlling Arm", isControllingArm);
            telemetry.addData("Slide Speed", "%4.2f", slideSpeed);
            telemetryAprilTag();
            telemetry.update();
        }

        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();


        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            builder.setCameraResolution(new Size(640, 480));
        }

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        //telemetry.addData("# AprilTag Info", currentDetections.toString());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                angles.setQuaternion(detection.metadata.fieldOrientation);

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Field Position: %s", detection.metadata.fieldPosition.toString()));
                //telemetry.addLine(String.format("Field Orientation: %s", detection.metadata.fieldOrientation.toString()));
                telemetry.addLine(String.format("Euler Angles: %s", angles.toString()));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                aprilTagDistance.put(0, (float) detection.ftcPose.x);
                aprilTagDistance.put(1, (float) detection.ftcPose.y);
                aprilTagDistance.put(2, (float) detection.ftcPose.z);

                relativeField.updateInfo(angles.yaw, aprilTagDistance);

                VectorF robotPose = detection.metadata.fieldPosition.subtracted(relativeField.getRelativeField());
                telemetry.addLine(String.format("Robot Position: %s", robotPose.toString()));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}
