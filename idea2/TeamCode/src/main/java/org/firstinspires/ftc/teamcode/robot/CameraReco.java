package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class CameraReco {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {"Ball","Cube","Duck","Marker"};
    private static final String VUFORIA_KEY = "AeR4Ufv/////AAABmSp+W7zrm0pWhA6GMhExjldWqRUlKMRyJSF9/NFxur3/ODgvpBoN9r5lsuZcdL14xnyvEtHRI5rgNAjpLrP4WK8XIMPsfpGpWKz3sk2tqcCZbNB2Z4Pgl/lqjvsEugzW3pwyJKVvLw1ndgrPSKjGjDySNf+aeVSnpenWRMBEKxRlqrTpzxtPyM/sUz0o+JJpxpWO2PUARj6Vtzifg1dqjz+07hMR5PjJMVtS8dPi7jc8INS/m4JSefls+PtybcpjhvulYTRPfOgZIEx9kqVpq4wUH8Yx+QIvKwkieyJq581KluslmbtO705kmuE5WCfoHLw/ugG5XCGYM44nLXsQjjBXEhFhVkkFgMsMwNH+EPKq";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public CameraReco(Telemetry globalTelemetry, HardwareMap hwm){
        telemetry = globalTelemetry;
        hardwareMap = hwm;
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        }
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public int duck_position(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                    i++;
                }
                telemetry.update();
            }
        }
        return 0;
    }
}
