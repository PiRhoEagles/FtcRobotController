package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Power Play Main Teleop", group="Linear Opmode")
//@Disabled
public class PowerPlayMainTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private DcMotor slideL = null;
    private DcMotor slideR = null;
    private Servo servoGrabber = null;

    // the power of the motors are multiplied by this
    double motorPowerFactor = 0.6;

    // the max power of the slides going up and down
    double maxSlidePowerUp = 0.5;
    double maxSlidePowerDown = 0.5;

    // one rotation of encoder tics for the slide motors
    double slideRotTics = 384.5;
    // mm in one slide motor rotation
    double mmPerSlideRotation = 116;
    // number of tics for one cm
    double slideTicsInCM = slideRotTics / (mmPerSlideRotation / 10);

    // the value of the grabber in its closed position
    double grabClosed = 0.24;
    // the value of the grabber in its open position
    double grabOpen = grabClosed + 0.16;
    // variable to store the position of the grabber servo; lower value is more closed
    double grabPosition = grabOpen;

    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------
        // adds telemetry that the robot has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize the motor hardware variables
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        slideL = hardwareMap.get(DcMotor.class, "slideL");
        slideR = hardwareMap.get(DcMotor.class, "slideR");

        // Most robots need the motors on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        slideR.setDirection(DcMotor.Direction.REVERSE);

        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // use braking to slow the motors down faster
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initialize the grabber servo variable
        servoGrabber = hardwareMap.get(Servo.class, "grabber");

        // initialize servo position
        servoGrabber.setPosition(grabPosition);

        // copies of the gamepad
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //---------------PRESSES PLAY---------------

            //----------CALC motorPowerFactor----------

            try {
                // Store the gamepad values from the previous loop iteration in
                // previousGamepad1 to be used in this loop iteration.
                // This is equivalent to doing this at the end of the previous
                // loop iteration, as it will run in the same order except for
                // the first/last iteration of the loop.
                previousGamepad1.copy(currentGamepad1);

                // Store the gamepad values from this loop iteration in
                // currentGamepad1 to be used for the entirety of this loop iteration.
                // This prevents the gamepad values from changing between being
                // used and stored in previousGamepad1/2.
                currentGamepad1.copy(gamepad1);
            }
            catch (com.qualcomm.robotcore.exception.RobotCoreException e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1 are being copied from valid Gamepads.
            }

            // rising edge detector for dpad_up
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                motorPowerFactor = Math.min(motorPowerFactor + .1, 1);
            }

            // rising edge detector for dpad_down
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                motorPowerFactor = Math.max(motorPowerFactor - .1, 0.1);
            }


            //----------CALC DRIVE MOTOR POWER----------

            // joystick values used to determine drive movement
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double powerFR = ((y - x - rx) / denominator) * motorPowerFactor;
            double powerFL = ((y + x + rx) / denominator) * motorPowerFactor;
            double powerBL = ((y - x + rx) / denominator) * motorPowerFactor;
            double powerBR = ((y + x - rx) / denominator) * motorPowerFactor;


            //----------SLIDE CONTROLS----------

            // decides power of the slides
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                slideL.setTargetPosition((int)(3 * slideTicsInCM));
                slideR.setTargetPosition((int)(3 * slideTicsInCM));
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideL.setPower(maxSlidePowerUp);
                slideR.setPower(maxSlidePowerUp);
            } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                slideL.setTargetPosition(0);
                slideR.setTargetPosition(0);
                slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideL.setPower(maxSlidePowerDown);
                slideR.setPower(maxSlidePowerDown);
            }


            //----------GRABBER CONTROLS----------

            // rising edge detector for grabber position
            if (currentGamepad1.a && !previousGamepad1.a) {
                if (grabPosition == grabOpen) {
                    grabPosition = grabClosed;
                } else {
                    grabPosition = grabOpen;
                }
            }


            //----------SET MOTOR & SERVO POWER----------

            // Send calculated power to wheels
            motorFR.setPower(powerFR);
            motorFL.setPower(powerFL);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);
            //slideL.setPower(powerSlides);
            //slideR.setPower(powerSlides);

            // Send calculated power to servo
            servoGrabber.setPosition(grabPosition);


            //----------TELEMETRY----------

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("motorPowerFactor", motorPowerFactor);
            telemetry.addData("grabPosition", grabPosition);
            telemetry.addData("slideL position", slideL.getCurrentPosition());
            telemetry.addData("slideR position", slideR.getCurrentPosition());
            telemetry.update();
        }
    }
}
