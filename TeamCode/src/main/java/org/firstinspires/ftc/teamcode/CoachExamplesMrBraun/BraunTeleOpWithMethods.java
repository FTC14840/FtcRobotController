// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Register class as TeleOp on Driver Station - Place your name first
@TeleOp(name ="Braun Test TeleOp")

// @Disabled

// Begin class and extend methods for LinearOpMode - Place your name first
public class BraunTeleOpWithMethods extends LinearOpMode {

    // Create a new instance of the hardware class
    BraunMethods robot = new BraunMethods();

    // Override the method runOpMode from LinearOpMode
    @Override
    public void runOpMode() throws InterruptedException {

        // Run method from hardware class
        robot.initHardware(this);
        //robot.initVuforiaTracking(this);
        robot.initVuforiaVision(this);
        robot.calibrateGyro(this);

        // Do this code block until play is pressed
        while (!isStarted()) {
            robot.vuforiaTelemetry();
        }

        // Wait for the drive to press play
        waitForStart();

        //robot.deactiveTfod();

        // Repeat this code once play is pressed until stop is pressed
        while (opModeIsActive()) {

            if (gamepad1.left_bumper) {
                robot.cruiseControl(400);

            } else if(gamepad1.right_bumper){
                robot.cruiseControl(800);

            } else {
                robot.manualDrive();
            }

            robot.moveRobot();
            robot.driveTelemetry(this);
        }
    }
}