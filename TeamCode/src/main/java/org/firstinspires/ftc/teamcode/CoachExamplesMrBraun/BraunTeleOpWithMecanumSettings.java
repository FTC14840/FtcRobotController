// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Register class as TeleOp on Driver Station - Place your name first
@TeleOp(name ="Braun Test TeleOp")

// @Disabled

// Begin class and extend methods for LinearOpMode - Place your name first
public class BraunTeleOpWithMecanumSettings extends LinearOpMode {

    // Create a new instance of the hardware class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            // robot.driveTelemetry(this);
        }

        // Wait for the drive to press play
        waitForStart();


        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

//            if (robot.targetsAreVisible() && gamepad1.left_bumper) {
//                robot.cruiseControl(robot.PRIMARYDISTANCE);
//
//            } else if(robot.targetsAreVisible() && gamepad1.right_bumper){
//                robot.cruiseControl(robot.SECONDARYDISTANCE);
//
//            } else {
//                robot.manualDrive();
//            }

            robot.manualDrive();
            robot.driveTelemetry(this);
            robot.moveRobot();
        }
    }
}