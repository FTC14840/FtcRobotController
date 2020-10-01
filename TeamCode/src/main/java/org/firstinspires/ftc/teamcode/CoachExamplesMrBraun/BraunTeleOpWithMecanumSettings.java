// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.Old.BraunVuforiaHardware;
import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.Old.BraunVuforiaNavigation;

// @Disabled

// Register class as TeleOp on Driver Station
@TeleOp(name ="Braun Test TeleOp")

// Begin class and extend methods for LinearOpMode
public class BraunTeleOpWithMecanumSettings extends LinearOpMode {

    final double TARGET_DISTANCE =  400.0;

    // Create a new instance of the hardware class
    BraunMecanumSettings robot = new BraunMecanumSettings();
    BraunVuforiaNavigation nav      = new BraunVuforiaNavigation();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        robot.activateTfod();
        nav.activateTracking();
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            nav.targetsAreVisible();
            robot.tfodTelemetry();
        }

        // Wait for the drive to press play
        waitForStart();

        robot.deactivedTfod();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            // Run these methods from the hardware setup to move the bot
            robot.manualDrive();
            robot.moveRobot();
            robot.driveTelemetry(this);

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