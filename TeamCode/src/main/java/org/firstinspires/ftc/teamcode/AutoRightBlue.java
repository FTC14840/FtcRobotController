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
        robot.prepareLauncher(740);
        robot.gyroForward(63,0.60,-30,0);
        robot.gyroLeft(.30,5,0);
        robot.raiseMagazine();
        robot.gyroLeftPowershot(.30, 0,1000);
        robot.launcherAutoPowershot(740);
        robot.gyroLeftPowershot(.30, -3.0,1000);
        robot.launcherAutoPowershot(740);
        robot.gyroLeftPowershot(.30, 5.0,1000);
        robot.launcherAutoPowershot(780);
        robot.signalBlueAlliance();
        robot.launcherOff();

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.gyroLeft(.50,22,0);
            robot.gyroForward(65, 1.0,22,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(40, 0.60,0,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.gyroLeft(.50,7,0);
            robot.gyroForward(30, 1.0,7,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroReverse(18, 0.60,0,0);

        } else {

            // Code for Default Zone A
            robot.gyroLeft(.70,69,0);
            robot.gyroForward(32, 1.0,69,0);
            robot.gyroRight(.70,0,0);
            Thread.sleep(1000);
            robot.dropBlueWobbleGoal();
            robot.gyroStrafeRight(2,.50,0,0);

        }

        Thread.sleep(1000);
        robot.initAuxiliaryControls();

    }
}