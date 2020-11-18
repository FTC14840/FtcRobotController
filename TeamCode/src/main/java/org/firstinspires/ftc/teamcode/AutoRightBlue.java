package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlue")

public class AutoRightBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    double launcherSpeed = .90; // Speed for Powershot

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.initTfodTelemetry();
        }

        waitForStart();

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 250);
         // robot.gyroReverse(12, .20, 0, 250);
         // robot.gyroStrafeLeft(12, .20, 0, 250);
         // robot.gyroStrafeRight(12, .20, 0, 250);
         // robot.gyroLeft(.20, 90, 250;
         // robot.gyroRight(.20, -90, 250);
         **/

        robot.tfodRunningTelemetry();
        robot.signalBlueAlliance();
        robot.gyroForward(70,.6,-8,50);
        robot.launcherPowershot(launcherSpeed);
        robot.raiseMagazine();
        robot.gyroLeftPowershot(.30,0,250);
        robot.shootLauncher();
        robot.gyroRightPowershot(.30,-4,250);
        robot.shootLauncher();
        robot.gyroRightPowershot(.30,-10,250);
        robot.shootLauncher();
        Thread.sleep(1000);
        robot.signalBlueAlliance();
        robot.launcherOff();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.gyroLeft(.50,45,0);
            robot.gyroForward(60, 1.0,45,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            robot.gyroReverse(60, 1.0,0,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.gyroLeft(.50,60,0);
            robot.gyroForward(20, 1.0,60,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            robot.gyroReverse(40, 1.0,0,0);

        } else {

            // Code for Zone A
            robot.gyroLeft(.50,70,0);
            robot.gyroForward(25, 1.0,70,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();

        }

        robot.lowerMagazine();

    }
}