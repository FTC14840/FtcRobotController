package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name ="AutoLeftRed")

@Disabled

public class AutoLeftRed extends LinearOpMode {

    MechWarriorCode robot = new MechWarriorCode();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initHardware(this);
        robot.initTfod(this);
        robot.calibrateGyro(this);

        while (!isStarted()) {
            robot.tfodInitTelemetry();
        }

        waitForStart();

        robot.stopTfod(this);

        /** Example Movements
         // robot.gyroForward(12, .20, 0, 500);
         // robot.gyroReverse(12, .20, 0, 500);
         // robot.gyroStrafeLeft(12, .20, 0, 500);
         // robot.gyroStrafeRight(12, .20, 0, 000);
         // robot.gyroLeft(.20, 90, 5000;
         // robot.gyroRight(.20, 90, 500);
         **/

        if (robot.getTfodDetected() == "Quad") {

            robot.tfodRunningTelemetry();
            robot.gyroLeft(.20, 90, 500);
            robot.gyroRight(.20, 90, 500);

        } else if (robot.getTfodDetected() == "Single") {

            robot.tfodRunningTelemetry();
            robot.gyroRight(.20, 90, 500);
            robot.gyroLeft(.20, 90, 500);

        } else {

            robot.tfodRunningTelemetry();
            robot.gyroForward(12, .20, 0, 500);
            robot.gyroReverse(12, .20, 0, 500);

        }
    }
}