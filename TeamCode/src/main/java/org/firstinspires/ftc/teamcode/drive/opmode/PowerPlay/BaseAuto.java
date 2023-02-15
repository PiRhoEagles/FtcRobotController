package org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

abstract class BaseAuto extends LinearOpMode {
    // the webcam
    OpenCvWebcam webcam;
    // the pipeline
    PowerPlayPipeline pipeline = new PowerPlayPipeline(telemetry);

    protected SampleMecanumDrive drive;
    protected ScoringMechanism mechanism = new ScoringMechanism();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        mechanism.init(hardwareMap, true);

        initCameraStuff();
        PowerPlayPipeline.detectionStates state = PowerPlayPipeline.detectionStates.ONE;
        while (!opModeIsActive() && !isStopRequested()) {
            mechanism.closeClaw();
            state = pipeline.getState();
            telemetry.addData("Detected State", state);
            telemetry.update();
        }

        TrajectorySequence sequence = trajectorySequenceBuilder(state);
        waitForStart();
        drive.followTrajectorySequenceAsync(sequence);
        while (opModeIsActive()) {
            drive.update();
        }
    }

    public abstract TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState);

    // Initializes the camera stuff.
    public void initCameraStuff() {
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
    }

    // Used to store custom Pose2D that uses degrees instead of radians.
    // This makes it easy to edit positions from the FTC Dashboard.
    public static class customPose2D {
        public double x;
        public double y;
        public double h;
        public double tan;

        public customPose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public customPose2D(double x, double y, double h, double tan) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.tan = tan;
        }
    }
}
