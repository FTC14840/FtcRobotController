// Package name
package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.BraunMecanumSettings;
import org.firstinspires.ftc.teamcode.MecanumSettings;

// @Disabled

// Register class as TeleOp on Driver Station
@TeleOp(name ="Jonathan Test TeleOp")

// Begin class and extend methods for LinearOpMode
public class JonathanTeleOpWithMecanumSettings extends LinearOpMode {



    // Create a new instance of the hardware class
    JonathanMecanumSettings robot = new JonathanMecanumSettings();

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
                robot.cruiseControl(robot.SECONDARYDISTANCE);

            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.navigationTelemetry();
        }
    }
}