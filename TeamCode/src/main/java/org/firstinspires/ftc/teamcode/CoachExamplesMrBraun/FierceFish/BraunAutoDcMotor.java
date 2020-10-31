package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BraunAutoDcMotor")

@Disabled

public class BraunAutoDcMotor extends LinearOpMode {

    DcMotor firstTestMotor;
    DcMotor secondTestMotor;
    DcMotor thirdTestMotor;
    DcMotor fourthTestMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        firstTestMotor = hardwareMap.get(DcMotor.class, "testMotor");  // or firstTestMotor = hardwareMap.dcMotor.get("testMotor");
        firstTestMotor.setDirection(DcMotor.Direction.REVERSE);  // or forward
        firstTestMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // or float

        waitForStart();

        // Running the motor for time... time based code is inconsistent
        firstTestMotor.setPower(1.0);
        sleep(5000);
        firstTestMotor.setPower(0);

        // Code stacking sucks...
        firstTestMotor.setPower(-0.5);
        secondTestMotor.setPower(-0.5);
        thirdTestMotor.setPower(-0.5);
        fourthTestMotor.setPower(-0.5);
        sleep(5000);
        firstTestMotor.setPower(0.0);
        secondTestMotor.setPower(0);
        thirdTestMotor.setPower(0);
        fourthTestMotor.setPower(0);

        // Instead, group motors together with methods/functions
        driveForward(.50);
        sleep(1000);
        stopDriving();

        tankLeft(.50);
        sleep(1000);
        stopDriving();

        // But it gets even better...
        tankRight(.50,5000);

    }

    public void driveForward(double power) {
        firstTestMotor.setPower(power);
        secondTestMotor.setPower(power);
        thirdTestMotor.setPower(power);
        fourthTestMotor.setPower(power);
    }

    public void tankLeft(double power) {
        firstTestMotor.setPower(power);
        secondTestMotor.setPower(-power);
        thirdTestMotor.setPower(power);
        fourthTestMotor.setPower(-power);
    }

    public void tankRight(double power, int pause) {
        firstTestMotor.setPower(power);
        secondTestMotor.setPower(-power);
        thirdTestMotor.setPower(power);
        fourthTestMotor.setPower(-power);
        sleep(pause);
        stopDriving();
    }

    public void stopDriving() {
        firstTestMotor.setPower(0);
        secondTestMotor.setPower(0);
        thirdTestMotor.setPower(0);
        fourthTestMotor.setPower(0);
    }



}
