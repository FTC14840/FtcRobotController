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
        robot.gyroForward(66,0.60,-30,50);
        robot.raiseMagazine();
        robot.prepareLauncher();
        robot.launcherPowershot(940);
        Thread.sleep(2000);
        robot.gyroRightPowershot(.30, 0.0,250);
        robot.shootAutoLauncher();
        robot.gyroRightPowershot(.30, -3.0,250);
        robot.shootAutoLauncher();
        robot.gyroRightPowershot(.30, -6.0,250);
        robot.shootAutoLauncher();
        Thread.sleep(1000);
        robot.signalBlueAlliance();
        robot.launcherOff();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.gyroLeft(.50,16,0);
            robot.gyroForward(64, 1.0,16,0);
            robot.gyroRightPowershot(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(45, 0.60,0,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.gyroLeft(.50,0,0);
            robot.gyroForward(30, 1.0,0,0);
            robot.gyroRightPowershot(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(20, 0.60,0,0);

        } else {

            // Code for Default Zone A
            robot.gyroLeft(.50,70,0);
            robot.gyroForward(30, 1.0,70,0);
            robot.gyroRightPowershot(.50,0,0);
            Thread.sleep(1000);
            robot.dropBlueWobbleGoal();

        }

        Thread.sleep(1000);
        robot.initAuxiliaryControls();

    }
}