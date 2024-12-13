package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.behaviors.DefaultModuleBehaviorCollector;
import org.firstinspires.ftc.teamcode.modules.robot.RobotArm;
import org.firstinspires.ftc.teamcode.modules.robot.RobotBase;
import org.firstinspires.ftc.teamcode.modules.robot.RobotLinearSlide;

@TeleOp(name = "RobotTeleop", group = "Teleop")
public class RobotTeleop extends LinearOpMode {

    private final DefaultModuleBehaviorCollector collector = new DefaultModuleBehaviorCollector();

    @Override
    public void runOpMode() {

        /// Tell the DefaultModuleBehaviorCollector to instantiate all methods
        collector.collect(RobotBase.class, RobotArm.class, RobotLinearSlide.class);

        waitForStart();
        while (opModeIsActive()) {
            collector.executeAll();
            sleep(50);
            idle();
        }
    }
}
