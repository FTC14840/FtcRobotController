// Package name
package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun;

// Imports

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// Begin hardware class
public class BraunMethods {

    // Define OpMode for bot hardware
    private LinearOpMode botOpMode;

    // Define DcMotors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Define global variables/fields for three axis motion
    private double driveAxial = 0;  // Positive is forward
    private double driveLateral = 0;  // Positive is right
    private double driveYaw = 0;  // Positive is Counterclockwise

    // Define global variables/fields for driver control
    private double speed = 1;
    private double direction = 1;
    private boolean aButtonPad1 = false;
    private boolean bButtonPad1 = false;

    // Constants for driver control
    private static final double HIGHSPEED = 1;
    private static final double LOWSPEED = .75;
    private static final double TURNSENSITIVITY = 1.5;

    private static final double TICKS = 1440; // goBulda = 537.6, AndyMark = 1120, Tetrix = 1440
    private static final double GEARREDUCTION = 1.0; // Greater than 1.0; Less than 1.0 if geared up
    private static final double WHEELDIAMETERINCHES = 4.0;
    private static final double TICKSTOINCHES = (TICKS * GEARREDUCTION) / (Math.PI * WHEELDIAMETERINCHES);

    private static final String VUFORIA_KEY = "ATqulq//////AAABmfYPXE+z1EORrVmv4Ppo3CcPktGk5mvdMnvPi9/T3DMYGc2mju8KUyG9gAB7pKlb9k9SZnM0YSq1JUZ6trE1ZKmMU8z5QPuhA/b6/Enb+XVGwmjrRjhMfNtUNgiZDhtsUvxr9fQP4HVjTzlz4pv0z3MeWZmkAgIN8T8YM0EFWrW4ODqYQmZjB0Nri2KKVM9dlOZ5udPfTZ9YvMgrCyxxG7O8P84AvwCAyXxzxelL4OfGnbygs0V60CQHx51gqrki613PT/9D1Q1io5+UbN6xAQ26AdYOTmADgJUGlfC2eMyqls4qAIoOj+pcJbm5ryF5yW9pEGHmvor1c9HlCFwhKxiaxw+cTu8AEaAdNuR65i/p";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private String tfodDetected = "None";

    public String getTfodDetected() {
        return tfodDetected;
    }

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;


    /**
     * Hardware Methods
     **/

    public void initHardware(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        // Remind the driver to keep the bot still during the hardware init... Specifically for the gyro.
        botOpMode.telemetry.log().add("Robot Initializing. Do Not Move The Bot...");
        botOpMode.telemetry.update();

        // Map global variables/ fields to config file on Robot Controller
        frontLeft = botOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = botOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = botOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = botOpMode.hardwareMap.get(DcMotor.class, "backRight");

        // Reverse motors if they don't drive forward
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        stopAndResetEncoder();
        runUsingEncoder();

        // Set all motor power to zero
        moveRobot(0, 0, 0);
    }

    public void initVuforia(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = botOpMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    public void initTfod(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        int tfodMonitorViewId = botOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", botOpMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        // The TensorFlow software will scale the input images from the camera to a lower resolution.
        // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
        // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        // should be set to the value of the images used to create the TensorFlow Object Detection model
        // (typically 1.78 or 16/9).

        // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
        tfod.setZoom(2.5, 1.78);

        if (tfod != null) {
            tfod.activate();
        }
    }

    public void stopTfod(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        tfod.deactivate();
    }

    public void calibrateGyro(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        navxMicro = botOpMode.hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (navxMicro.isCalibrating()) {
            botOpMode.telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            botOpMode.telemetry.update();
        }

//        Thread.sleep(500);
//        botOpMode.telemetry.log().clear();
//        botOpMode.telemetry.log().add("Gyro Calibrated");
//        botOpMode.telemetry.update();
    }

    /**
     * TeleOp Methods
     **/

    public void manualDrive() {
        // Setting three motions to stick movements
        setAxial(-botOpMode.gamepad1.left_stick_y);
        setLateral(botOpMode.gamepad1.left_stick_x);
        setYaw(-botOpMode.gamepad1.right_stick_x);

        // Logic for speed control on button A
        if (botOpMode.gamepad1.a) {
            aButtonPad1 = true;
        } else if (aButtonPad1) {
            aButtonPad1 = false;
            if (speed == LOWSPEED) {
                speed = HIGHSPEED;
            } else {
                speed = LOWSPEED;
            }
        }

        // Logic for direction control on button B
        if (botOpMode.gamepad1.b) {
            bButtonPad1 = true;
        } else if (bButtonPad1) {
            bButtonPad1 = false;
            direction = -direction;
        }
    }

    // Clipping motor power for axial motion
    public void setAxial(double axial) {
        driveAxial = Range.clip(axial, -1, 1);
    }

    // Clipping motor power for lateral motion
    public void setLateral(double lateral) {
        driveLateral = Range.clip(lateral, -1, 1);
    }

    // Clipping motor power for yaw motion
    public void setYaw(double yaw) {
        driveYaw = Range.clip(yaw, -1, 1);
    }

    // Calculations for the power to each motor
    public void moveRobot() {
        // Vector addition and algebra for the pwer to each motor
        double moveFrontLeft = speed * (direction * ((-driveAxial) + (driveLateral)) + (driveYaw * TURNSENSITIVITY));
        double moveFrontRight = speed * (direction * ((-driveAxial - driveLateral)) - (driveYaw * TURNSENSITIVITY));
        double moveBackLeft = speed * (direction * ((-driveAxial - driveLateral)) + (driveYaw * TURNSENSITIVITY));
        double moveBackRight = speed * (direction * ((-driveAxial + driveLateral)) - (driveYaw * TURNSENSITIVITY));

        // Normalize all motor speeds so no value exceeds 100% power.
        double max = Math.max(Math.abs(moveFrontLeft), Math.abs(moveFrontRight));
        max = Math.max(max, Math.abs(moveBackLeft));
        max = Math.max(max, Math.abs(moveBackRight));

        // Proportional logic to reduce all motors to the highest value
        if (max > 1.0) {
            moveFrontLeft /= max;
            moveFrontRight /= max;
            moveBackLeft /= max;
            moveBackRight /= max;
        }

        // Set drive motor power based on calculations
        frontLeft.setPower(moveFrontLeft);
        frontRight.setPower(moveFrontRight);
        backLeft.setPower(moveBackLeft);
        backRight.setPower(moveBackRight);
    }

    // Overloaded Method to set all three motions and move the robot
    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setLateral(lateral);
        setYaw(yaw);
        moveRobot();
    }

    /**
     * Autonomous Methods
     **/

    public void gyroForward(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            driveByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power - error);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void driveByInches(int distance) {
        frontLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
    }

    public void gyroReverse(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            driveByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power + error);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroStrafeLeft(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            strafeByInches(-distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power + error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power - error);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void strafeByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) TICKSTOINCHES);
        frontRight.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backLeft.setTargetPosition(-distance * (int) TICKSTOINCHES);
        backRight.setTargetPosition(distance * (int) TICKSTOINCHES);
    }

    public void gyroStrafeRight(int distance, double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            strafeByInches(distance);
            runToPosition();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power - error);
                    backLeft.setPower(power + error);
                    backRight.setPower(power + error);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroLeft(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading > angle) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void gyroRight(double power, double angle, int pause) throws InterruptedException {
        if (botOpMode.opModeIsActive()) {
            stopAndResetEncoder();
            runUsingEncoder();
            while (botOpMode.opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading < angle) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                    // driveTelemetry(botOpMode);
                } else {
                    stopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void stopAndResetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Telemetry Methods
     **/

    public void driveTelemetry(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        if (botOpMode.gamepad1.y) {
            stopAndResetEncoder();
            runUsingEncoder();
        }

        double frontLeftInches = frontLeft.getCurrentPosition() / TICKSTOINCHES;
        double frontRightInches = frontRight.getCurrentPosition() / TICKSTOINCHES;
        double backLeftInches = backLeft.getCurrentPosition() / TICKSTOINCHES;
        double backRightInches = backRight.getCurrentPosition() / TICKSTOINCHES;
        double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        botOpMode.telemetry.log().clear();
        botOpMode.telemetry.addData("Note", "--> Tap Y to reset encoders");
        botOpMode.telemetry.addData("Heading", "%.2f", gyroHeading);
        botOpMode.telemetry.addData("Inches", "FL: %2d, FR: %2d, BL: %2d, BR: %2d", -(int) frontLeftInches, -(int) frontRightInches, -(int) backLeftInches, -(int) backRightInches);
        botOpMode.telemetry.addData("Encoder", "FL: %2d,  FR: %2d, BL: %2d, BR: %2d", -frontLeft.getCurrentPosition(), -frontRight.getCurrentPosition(), -backLeft.getCurrentPosition(), -backRight.getCurrentPosition());
        botOpMode.telemetry.addData("Target", "FL: %2d, FR: %2d, BL: %2d, BR: %2d", -frontLeft.getTargetPosition(), -frontRight.getTargetPosition(), -backLeft.getTargetPosition(), -backRight.getTargetPosition());
        botOpMode.telemetry.addData("Power", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", frontLeft.getPower(), frontRight.getPower(), backLeft.getPower(), backRight.getPower());
        botOpMode.telemetry.addData("Axes", "A: %.2f, L: %.2f, Y: %.2f", driveAxial, driveLateral, driveYaw);
        botOpMode.telemetry.update();
    }

    public void tfodTelemetry(LinearOpMode opMode) throws InterruptedException {

        // Set opMode to one defined above
        botOpMode = opMode;

        if (botOpMode.gamepad1.y) {
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Note", "Resetting TFOD");
            botOpMode.telemetry.update();
            tfod.deactivate();
            tfod.activate();
            Thread.sleep(1000);
        }

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                tfodDetected = recognition.getLabel();
                botOpMode.telemetry.log().clear();
                botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
                botOpMode.telemetry.addData("Note", "--> Tap Y to Reset TFOD");
                botOpMode.telemetry.addData(String.format("Detected (%d)", i), recognition.getLabel());
                botOpMode.telemetry.update();
                Thread.sleep(5000);
            }
        } else {
            tfodDetected = "None";
            botOpMode.telemetry.log().clear();
            botOpMode.telemetry.addData("Robot Initialized", "Press Play to Begin");
            botOpMode.telemetry.addData("Note", "--> Tap Y to Reset TFOD");
            botOpMode.telemetry.addData("Detected", "None");
            botOpMode.telemetry.update();
        }
    }
}