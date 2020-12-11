package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="MeasureTeleOp")

@Disabled

public class MeasureTeleOp extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.initTelemetry();
        }

        waitForStart();

        while (opModeIsActive()) {

            robot.manualDrive();
            robot.moveRobot();
            robot.driveTelemetry();
        }
    }
}
