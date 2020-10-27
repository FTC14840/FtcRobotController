package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FinalTeleOp")

public class FinalTeleOp extends LinearOpMode {

    final double TARGET_DISTANCE =  1524.0;

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initVisionTracking(this);
        robot.activateCruiseControl(this);

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(TARGET_DISTANCE);
            } else {
                robot.manualDrive();
            }

            robot.cruiseControlTelemetry();
            robot.moveRobot();
            telemetry.update();
        }
    }
}
