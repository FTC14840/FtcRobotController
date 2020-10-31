package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BraunAutoBlinkinEndGameTimer")

//@Disabled

public class BraunAutoBlinkinEndGameTimer extends LinearOpMode {

    RevBlinkinLedDriver ledLights;
    int blinkinTimer = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
        ledLights = hardwareMap.get(RevBlinkinLedDriver.class, "ledLights");
        ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        waitForStart();

        if (blinkinTimer == 0){
            resetStartTime();
            blinkinTimer = 1;
        }

        while(opModeIsActive()) {

            if(time >= 90 && time < 110){
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY);
            } else if(time >= 110 && time < 120) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
            } else {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
    }
}
