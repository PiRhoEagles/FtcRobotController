package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pipelines.PowerPlayBlueSidePipeline.detectionStates.ONE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayBlueSidePipeline;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class PowerPlayBlueSideAuto extends LinearOpMode {
    // the webcam
    OpenCvWebcam webcam;
    // the pipeline
    PowerPlayBlueSidePipeline pipeline = new PowerPlayBlueSidePipeline(telemetry);

    @Override
    public void runOpMode() {
        // Instantiate an OpenCvCamera object for the camera we'll be using.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Specify the pipeline we want to use.
        webcam.setPipeline(pipeline);

        // Open the connection to the camera device.
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Tell the webcam to start streaming images to us.
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened.
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Initializes the state variable.
        PowerPlayBlueSidePipeline.detectionStates state = ONE;

        // Repeatedly detects the state until play is hit.
        // If this runs, init has already been hit, so it'll
        // run until the play button is hit.
        while (!opModeIsActive() && !isStopRequested()) {
            state = pipeline.getState();
            telemetry.addData("Detected State", state);
            telemetry.update();
        }

        // Just skips right over since play has already been hit.
        waitForStart();

        // Stops streaming the webcam since we don't need it anymore.
        // This also stops calling the pipeline.
        webcam.stopStreaming();

        while (opModeIsActive()) {
            // Send telemetry things.
            telemetry.addData("Detected State", state);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();


            // Throttle ourselves to 10Hz loop to avoid burning excess CPU cycles for no reason.
            // In a real OpMode you might not want to do this.
            sleep(100);
        }
    }
}