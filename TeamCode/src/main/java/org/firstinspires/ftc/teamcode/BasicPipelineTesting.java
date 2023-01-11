package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline.detectionStates.ONE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class BasicPipelineTesting extends LinearOpMode {
    // Whether or not this is the first run.
    boolean firstRun = true;

    // the webcam
    OpenCvWebcam webcam;
    // the pipeline
    PowerPlayPipeline pipeline = new PowerPlayPipeline(telemetry);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor slideL = null;
    private DcMotor slideR = null;
    private Servo grabberL = null;
    private Servo grabberR = null;


    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------
        // Initializes the motors and servos.
        initMotorsAndServos();

        // Initializes the camera stuff.
        initCameraStuff();

        // Initializes the state variable.
        PowerPlayPipeline.detectionStates state = ONE;

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


        while (opModeIsActive()) { //---------------MAIN LOOP AFTER PLAY---------------
            if (firstRun) {
                firstRun = false;

                switch (state) {
                    case ONE:
                        runStateONE();
                        break;
                    case TWO:
                        runStateTWO();
                        break;
                    case THREE:
                        runStateTHREE();
                        break;
                }
            }

            // Send telemetry things.
            //telemetry.addData("Detected State", state);
            //telemetry.update();
        }
    }


    // Initialize the motors and servos.
    public void initMotorsAndServos() {
        // initialize the motor hardware variables
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        slideL = hardwareMap.get(DcMotor.class, "slideL");
        slideR = hardwareMap.get(DcMotor.class, "slideR");

        // reverses some of the motor directions
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        slideR.setDirection(DcMotor.Direction.REVERSE);

        // use braking to slow the drive motors down faster
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initialize the grabber servos variable
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");
    }

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

    // Runs if it's state ONE.
    public void runStateONE() {
        // Move left.
        motorFR.setPower(0.5);
        motorFL.setPower(-0.5);
        motorBL.setPower(0.5);
        motorBR.setPower(-0.5);

        sleep(500);

        // Stop moving.
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // Runs if it's state TWO.
    public void runStateTWO() {
        // Move forward.
        motorFR.setPower(0.5);
        motorFL.setPower(0.5);
        motorBL.setPower(0.5);
        motorBR.setPower(0.5);

        sleep(500);

        // Stop moving.
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    // Runs if it's state THREE.
    public void runStateTHREE() {
        // Move right.
        motorFR.setPower(-0.5);
        motorFL.setPower(0.5);
        motorBL.setPower(-0.5);
        motorBR.setPower(0.5);

        sleep(500);

        // Stop moving.
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}