package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoLeftBlue")

// @Disabled

public class AutoLeftBlue extends LinearOpMode {

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
         robot.gyroForward(12, .50, 0, 500);
         robot.gyroReverse(12, .50, 0, 500);
         robot.gyroStrafeLeft(12, .50, 0, 500);
         robot.gyroStrafeRight(12, .50, 0, 500);
         robot.gyroLeft(.50, 90, 500);
         robot.gyroRight(.50, -90, 500);
         robot.gyroLeft(.50,0,500);
         **/

        if (robot.getTfodDetected() == "Quad") {

            robot.tfodRunningTelemetry();
            robot.gyroLeft(.50, 90, 500);

        } else if (robot.getTfodDetected() == "Single") {

            robot.tfodRunningTelemetry();
            robot.gyroRight(.50, -90, 500);

        } else {

            robot.tfodRunningTelemetry();
            robot.gyroReverse(100, .50, 0, 500);


//             robot.gyroForward(12, .50, 0, 500);
//             robot.gyroReverse(12, .50, 0, 500);
//             robot.gyroStrafeLeft(12, .50, 0, 500);
//             robot.gyroStrafeRight(12, .50, 0, 500);
//             robot.gyroLeft(.50, 90, 500);
//             robot.gyroRight(.50, -90, 500);
//             robot.gyroLeft(.50,0,500);

        }
    }
}




