package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BraunAutoColorSensor")

@Disabled

public class BraunAutoColorSensor extends LinearOpMode {

    DcMotor leftDriveMotor;
    DcMotor rightDriveMotor;
    BNO055IMU imu;
    Orientation angles;

    ColorSensor colors;

    @Override
    public void runOpMode() throws InterruptedException {

        leftDriveMotor = hardwareMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor = hardwareMap.dcMotor.get("rightDriveMotor");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        colors = hardwareMap.get(ColorSensor.class, "colors");

        waitForStart();

        /**
         * Use red, green, blue, or alpha.  Conbined (argb) is not recommended.
         */

        driveForward(.5,1000,0);
        sleep(1000);
        if(colors.alpha() >1000){
            // Do this code
        } else {
            // Do this code instead
        }


    }

    void driveForward (double power, double ticks, double targetAngle) throws InterruptedException {
        double leftPower;
        double rightPower;
        leftDriveMotor.setPower(power);
        rightDriveMotor.setPower(power);
        while(leftDriveMotor.getCurrentPosition() < ticks && rightDriveMotor.getCurrentPosition() < ticks){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(angles.firstAngle < targetAngle){
                leftDriveMotor.setPower(power + .05);  // Instead, look into using PID for the .05 value.
                rightDriveMotor.setPower(power - .05);
            } else if(angles.firstAngle > targetAngle) {
                leftDriveMotor.setPower(power - .05);
                rightDriveMotor.setPower(power +.05);
            } else {
                leftDriveMotor.setPower(power);
                rightDriveMotor.setPower(power);
            }
        }
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        resetEncoders();
    }

    void turnleft (double turnAngle, double timeouts){
        sleep(250);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double speed = 0.5;
        double oldDegreesLeft = turnAngle;
        double scaleSpeed = speed;
        double targetHeading = angles.firstAngle + turnAngle;
        double oldAngle = angles.firstAngle;
        if(targetHeading < -180){
            targetHeading += 360;
        }
        if (targetHeading > 180) {
            targetHeading -= 360;
        }
        double degreesLeft = ((Math.signum(angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading)) +
                (Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        resetStartTime();
        while(opModeIsActive() && time < timeouts && degreesLeft > 1 && oldDegreesLeft-degreesLeft >= 0){
            scaleSpeed = degreesLeft / (100 + degreesLeft) * speed;
            if(scaleSpeed > 1){
                scaleSpeed = .1;
            }
            leftDriveMotor.setPower(scaleSpeed);
            rightDriveMotor.setPower(-scaleSpeed);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft = degreesLeft;
            degreesLeft = ((Math.signum(angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading)) +
                    (Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle) < 1) {
                speed *= 1.1;
            }
            oldAngle = angles.firstAngle;
        }
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        resetEncoders();
    }

    void resetEncoders(){

        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Might not need this?
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
