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



    // Create a new instance of the hardware class
    BraunMecanumSettings robot = new BraunMecanumSettings();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        robot.initVision(this);
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            robot.targetsAreVisible();
        }

        // Wait for the drive to press play
        waitForStart();

        robot.deactiveTfod();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
                robot.cruiseControl(robot.PRIMARYDISTANCE);

            } else if(robot.targetsAreVisible() && gamepad1.right_bumper){
                robot.cruiseControl(robot.SECONDAYDISTANCE);

            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.navigationTelemetry();
        }
    }
}