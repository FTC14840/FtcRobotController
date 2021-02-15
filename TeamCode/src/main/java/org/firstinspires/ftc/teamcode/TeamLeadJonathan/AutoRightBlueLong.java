package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MechWarriorCode;

@Autonomous (name ="AutoRightBlueLong")

public class AutoRightBlueLong extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);
        robot.magazineSetup(.02, 2000);

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

        if (robot.getTfodDetected() == "Quad") {

            // Code for Zone C
            robot.prepareLauncher(900);
            robot.gyroForward(25, 0.70, 25, 0);
            robot.moveRobot(0,0,0);
            robot.gyroLeft(.30,3,0);
            robot.gyroRight(.30,3,0);
            robot.moveRobot(0,0,0);
            robot.dropBlueWobbleGoal();
            robot.autoIntakeOn();
            robot.autoLowerIntake(.85);
            Thread.sleep(500);
            robot.autoIntakeOff();
            robot.raiseMagazine();
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            robot.lowerMagazine();
            robot.autoIntakeOn();
            Thread.sleep(4000);
            robot.autoIntakeOff();
            robot.raiseMagazine();
            robot.autoLowerIntake(.95);
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            robot.autoIntakeOn();
            robot.lowerMagazine();
            robot.gyroForward(50,1.0, 7.5,0);

        } else if (robot.getTfodDetected() == "Single") {

            // Code for Zone B
            robot.prepareLauncher(900);
            robot.gyroForward(26, 0.70, 25, 0);
            robot.moveRobot(0,0,0);
            robot.gyroLeft(.30,3,0);
            robot.gyroRight(.30,3,0);
            robot.moveRobot(0,0,0);
            robot.autoIntakeOn();
            robot.autoLowerIntake(.95);
            Thread.sleep(500);
            robot.autoIntakeOff();
            robot.raiseMagazine();
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            robot.lowerMagazine();
            robot.autoIntakeOn();
            Thread.sleep(2000);
            robot.autoIntakeOff();
            robot.raiseMagazine();
            robot.autoLowerIntake(.75);
            robot.launcherAutoPowershot(900);
            Thread.sleep(250);
            robot.launcherAutoPowershot(900);
            robot.gyroRight(.40,-10,0);
            robot.gyroForward(64,.70, -10,0);
            robot.dropBlueWobbleGoal();
            robot.gyroLeft(.40,20,0);
            robot.gyroReverse(10,.70,20,0);
            robot.gyroRight(.40,-5,0);
            robot.lowerMagazine();

        } else {

            // Code for Default Zone A
            robot.prepareLauncher(820);
            robot.gyroForward(60, .50, 5, 0);
            robot.moveRobot(0,0,0);
            robot.gyroLeft(.25,4,0);
            robot.gyroRight(.25,4,0);
            robot.moveRobot(0,0,0);
            robot.raiseMagazine();
            Thread.sleep(11000);
            robot.launcherAutoPowershot(820);
            Thread.sleep(1000);
            robot.launcherAutoPowershot(820);
            Thread.sleep(1000);
            robot.launcherAutoPowershot(820);
            robot.gyroLeft(.25,30,0);
            robot.gyroForward(16,.70,30,0);
            robot.dropBlueWobbleGoal();
            Thread.sleep(1000);
            robot.gyroLeft(.40,85,0);
            robot.gyroReverse(22,.70,85,0);
            robot.gyroRight(.40,5,0);
            robot.lowerMagazine();
            robot.autoLowerIntake(.95);

        }

        robot.initAuxiliaryControls();
        Thread.sleep(30000);

    }
}