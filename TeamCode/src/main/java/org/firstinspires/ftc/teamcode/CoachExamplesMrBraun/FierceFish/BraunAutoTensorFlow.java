package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "BraunAutoTensorFlow")

@Disabled

public class BraunAutoTensorFlow extends LinearOpMode {

    // https://developer.vuforia.com/license-manager
    private static final String VUFORIA_KEY = "ATqulq//////AAABmfYPXE+z1EORrVmv4Ppo3CcPktGk5mvdMnvPi9/T3DMYGc2mju8KUyG9gAB7pKlb9k9SZnM0YSq1JUZ6trE1ZKmMU8z5QPuhA/b6/Enb+XVGwmjrRjhMfNtUNgiZDhtsUvxr9fQP4HVjTzlz4pv0z3MeWZmkAgIN8T8YM0EFWrW4ODqYQmZjB0Nri2KKVM9dlOZ5udPfTZ9YvMgrCyxxG7O8P84AvwCAyXxzxelL4OfGnbygs0V60CQHx51gqrki613PT/9D1Q1io5+UbN6xAQ26AdYOTmADgJUGlfC2eMyqls4qAIoOj+pcJbm5ryF5yW9pEGHmvor1c9HlCFwhKxiaxw+cTu8AEaAdNuR65i/p";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private String tfodDetected = "None";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        initVuforia();
        initTfod();

        while (!isStarted()) {
            tfodTelemetry();
            // Add GREEN LED's to signal ready
        }

        waitForStart();

        stopTfod();

        while (opModeIsActive()) {

            if (tfodDetected == "Quad") {

                telemetry.log().clear();
                telemetry.addData("Running", "Quad Program");
                telemetry.update();

                // Do this code...

            } else if (tfodDetected == "Single") {

                telemetry.log().clear();
                telemetry.addData("Running", "Single Program");
                telemetry.update();

                // Do this code...

            } else {

                telemetry.log().clear();
                telemetry.addData("Running", "Default Program");
                telemetry.update();

                // Do this code...

            }
        }
    }

    /**
     * Methods defined below...
     */

    private void initVuforia() {

        VuforiaLocalizer.Parameters vuforiaParameters = new VuforiaLocalizer.Parameters();
        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.setZoom(1.0, 1.78);
        if (tfod != null) {
            tfod.activate();
        }
    }

    private void stopTfod() {
        tfod.deactivate();
        tfod.shutdown();
    }

    private void initHardware() {
        telemetry.log().clear();
        telemetry.log().add("Robot Initializing. Do Not Move The Bot...");
        telemetry.update();
    }

    public void tfodTelemetry() throws InterruptedException {

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                tfodDetected = recognition.getLabel();
                telemetry.log().clear();
                telemetry.addData("Robot Initialized", "Press Play to Begin");
                telemetry.addData(String.format("Detected (%d)", i), recognition.getLabel());
                telemetry.update();
                sleep(1000);
            }
        } else {
            tfodDetected = "None";
            telemetry.log().clear();
            telemetry.addData("Robot Initialized", "Press Play to Begin");
            telemetry.addData("Detected", "None");
            telemetry.update();
            sleep(1000);
        }
    }
}
