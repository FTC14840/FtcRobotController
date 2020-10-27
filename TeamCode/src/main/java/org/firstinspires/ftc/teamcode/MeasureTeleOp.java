package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MeasureTeleOp")

public class MeasureTeleOp extends LinearOpMode {

    final double TARGET_DISTANCE =  1524.0;

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.calibrateGyro(this);


        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            robot.manualDrive();
            robot.moveRobot();
            robot.driveTelemetry();
            telemetry.update();
        }
    }
}
