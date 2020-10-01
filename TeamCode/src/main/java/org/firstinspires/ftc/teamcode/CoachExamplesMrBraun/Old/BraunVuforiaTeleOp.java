package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Vuforia TeleOp")

@Disabled

public class BraunVuforiaTeleOp extends LinearOpMode {

    final double TARGET_DISTANCE =  400.0;

    BraunVuforiaHardware robot    = new BraunVuforiaHardware();
    BraunVuforiaNavigation nav      = new BraunVuforiaNavigation();

    @Override
    public void runOpMode() {

        robot.initDrive(this);
        nav.initVuforia(this, robot);

        nav.activateTracking();

        while (!isStarted()) {

            telemetry.addData(">", "Press start");

            nav.targetsAreVisible();
            nav.addNavTelemetry();
            telemetry.update();
        }

        while (opModeIsActive()) {

            telemetry.addData(">", "Press Left Bumper to track target");

            if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                nav.cruiseControl(TARGET_DISTANCE);

            } else {
                robot.manualDrive();
            }

            nav.addNavTelemetry();

            robot.moveRobot();
            telemetry.update();
        }
    }
}

