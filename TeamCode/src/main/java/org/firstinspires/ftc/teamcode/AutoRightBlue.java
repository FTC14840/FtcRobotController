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
        robot.magazineSetup(.02,2000);

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
        robot.prepareLauncher(880);
        robot.gyroForward(64,0.60,-30,0);
        robot.gyroLeft(.30,0,0);
        robot.raiseMagazine();
        robot.gyroRightPowershot(.30, 0,1000);
        robot.launcherAutoPowershot(880);
        robot.gyroRightPowershot(.30, -5.0,1000);
        robot.launcherAutoPowershot(880);
        robot.gyroRightPowershot(.30, 4.0,1000);
        robot.launcherAutoPowershot(880);
        robot.signalBlueAlliance();
        robot.launcherOff();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.gyroLeft(.50,20,0);
            robot.gyroForward(54, 1.0,20,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(45, 0.60,0,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.gyroLeft(.50,0,0);
            robot.gyroForward(30, 1.0,0,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(20, 0.60,0,0);

        } else {

            // Code for Default Zone A
            robot.gyroLeft(.50,57,0);
            robot.gyroForward(32, 1.0,70,0);
            robot.gyroRight(.50,0,0);
            Thread.sleep(1000);
            robot.dropBlueWobbleGoal();
            robot.gyroRight(.50,-20,0);

        }

        Thread.sleep(1000);
        robot.initAuxiliaryControls();

    }
}