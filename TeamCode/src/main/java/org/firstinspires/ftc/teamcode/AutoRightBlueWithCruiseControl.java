package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlueWithCruiseControl")

//@Disabled

public class AutoRightBlueWithCruiseControl extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double LAUNCHER_SPEED = 0.80;
    final double POWERSHOT_RANGE = 1700;
    final double POWERSHOT_OFFSET = 500;
    final double POWERSHOT_ANGLE = -10;
    final double POWERSHOT_RANGE_AXIAL_GAIN = 0.0030;
    final double POWERSHOT_RANGE_LATERAL_GAIN = 0; //.0030;
    final double POWERSHOT_RANGE_YAW_GAIN = 0.0400;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.initTfodTelemetry();
        }

        waitForStart();

        robot.tfodRunningTelemetry();
        robot.signalBlueAlliance();
        robot.launcherPowerUp();
        robot.raiseMagazine();

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 500);
         // robot.gyroReverse(12, .20, 0, 500);
         // robot.gyroStrafeLeft(12, .20, 0, 500);
         // robot.gyroStrafeRight(12, .20, 0, 000);
         // robot.gyroLeft(.20, 90, 5000;
         // robot.gyroRight(.20, -90, 500);
         **/

//            while (robot.targetsAreVisible() && opModeIsActive()) {
//
//                robot.powerShot(POWERSHOT_RANGE, POWERSHOT_OFFSET, POWERSHOT_ANGLE, POWERSHOT_RANGE_AXIAL_GAIN, POWERSHOT_RANGE_LATERAL_GAIN, POWERSHOT_RANGE_YAW_GAIN);
//                robot.moveRobot();
//                robot.ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//                robot.cruiseControlTelemetry();
//                if(robot.readyToShot(POWERSHOT_RANGE)) {
//                    robot.ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                    break;
//                }
//            }
//
//            sleep(30000);


        robot.gyroForward(12, 1.0, 0,0);
        robot.gyroRight(.50,-45,0);
        robot.gyroForward(20,1.0,-45, 0);
        robot.gyroLeft(.50, 0,0);
        robot.gyroForward(60, 1.0,0,250);
        robot.gyroLeftPowershot(.30,0,250);
        robot.shootLauncher();
        robot.gyroRightPowershot(.30,-5,250);
        robot.shootLauncher();
        robot.gyroRightPowershot(.30,-10,250);
        robot.shootLauncher();
        robot.signalBlueAlliance();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B

        } else {

            // Code for Zone A
            robot.gyroLeft(.50,90,0);
            robot.gyroForward(30, 1.0,90,0);
            robot.gyroRight(.50,0,0);
            // Drop Wobble Goal
            robot.gyroReverse(30, 1.0,0,0);
            // Park





        }
    }
}