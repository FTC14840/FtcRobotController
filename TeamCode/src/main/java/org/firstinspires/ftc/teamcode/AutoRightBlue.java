package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlue")

public class AutoRightBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

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
        robot.gyroForward(70,.60,-8,50);
        robot.raiseMagazine();
        robot.shootLauncher();
        robot.launcherPowershot(0.82);
        robot.gyroRight(.30, 0.0,250);
        robot.shootAutoLauncher();
        robot.gyroRightPowershot(.30, -5.0,250);
        robot.shootAutoLauncher();
        robot.gyroRightPowershot(.30, -10.0,250);
        robot.shootAutoLauncher();
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

        Thread.sleep(1000);
        robot.lowerMagazine();

    }
}