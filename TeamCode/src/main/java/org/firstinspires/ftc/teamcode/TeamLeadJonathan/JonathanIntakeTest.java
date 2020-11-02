package org.firstinspires.ftc.teamcode.TeamLeadJonathan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class JonathanIntakeTest extends LinearOpMode {
    JonathanMethods intake = new JonathanMethods();
    public void runOpMode() throws InterruptedException{
        intake.iniitIntakeHardwareOnly();
        waitForStart();
        intake.startIntake();
        while (opModeIsActive()){
            intake.adjustIntake();
            intake.intakeTelemetry(this);
        }
    }
}
