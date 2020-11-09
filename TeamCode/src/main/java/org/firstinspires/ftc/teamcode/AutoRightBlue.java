package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoRightBlue")

//@Disabled

public class AutoRightBlue extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    final double LAUNCHER_SPEED = 0.80;

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
        robot.launcherPowerUp(LAUNCHER_SPEED);
        robot.raiseMagazine();

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 250);
         // robot.gyroReverse(12, .20, 0, 250);
         // robot.gyroStrafeLeft(12, .20, 0, 250);
         // robot.gyroStrafeRight(12, .20, 0, 250);
         // robot.gyroLeft(.20, 90, 250;
         // robot.gyroRight(.20, -90, 250);
         **/

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
            robot.gyroLeft(.50,90,0);
            robot.gyroForward(40, 1.0,90,0);
            robot.gyroRight(.50,0,0);
            robot.dropBlueWobbleGoal();
            robot.gyroReverse(20, 1.0,0,0);

        }

        robot.lowerMagazine();

    }
}