package org.firstinspires.ftc.teamcode.CoachExamplesMrBraun.FierceFish;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "BraunAutoVuforia")

@Disabled

public class BraunAutoVuforia extends LinearOpMode {

    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;
    VuforiaTrackables visionTargets;
    VuforiaTrackable target;
    VuforiaTrackableDefaultListener listener;
    OpenGLMatrix lastKnownLocation;
    OpenGLMatrix phoneLocation;

    @Override
    public void runOpMode() throws InterruptedException {

        vuforiaSetup();

        lastKnownLocation = createMatrix(0,0,0,0,0,0);

        waitForStart();

        visionTargets.activate();

        while(opModeIsActive()){

            OpenGLMatrix lastLocation = listener.getUpdatedRobotLocation();

            if (lastLocation != null){
                lastKnownLocation = lastLocation;
            }

            telemetry.addData("Tracking" + target.getName(),listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

        }

    }

    void vuforiaSetup(){

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        visionTargets = vuforia.loadTrackablesFromAsset("Skystone");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);
        target = visionTargets.get(9);
        target.setName("Blue Perimeter 1");
        target.setLocation(createMatrix(0,1000,0,90,0,90));
        phoneLocation = createMatrix (0,225,0,90,0,0);
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation,parameters.cameraDirection);

    }

    OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w){
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, u, v, w));
    }

    String formatMatrix (OpenGLMatrix matrix) {
        return matrix.formatAsTransform();
    }
}

