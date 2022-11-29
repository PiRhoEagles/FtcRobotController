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
    private Servo grabberL = null;
    private Servo grabberR = null;


    // the power of the motors are multiplied by this
    double motorPowerFactor = 0.6;


    // how many encoder tics make one full slide motor rotation
    static final double SLIDE_TICS_IN_ROT = 384.5;
    // the number of mm the slides move from one motor rotation
    static final double SLIDE_MM_FROM_ROT = 116;
    // number of tics to move slide by 1cm
    static final double SLIDE_TICS_IN_CM = SLIDE_TICS_IN_ROT / (SLIDE_MM_FROM_ROT / 10);

    // the slide's level; 0-3, 0 being ground, 3 being highest pole
    int slideLvl = 0;

    // whether or not the grabber is open
    boolean grabberOpen = true;


    // copies of the gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    @Override
    public void runOpMode() { //---------------PRESSES INITIALIZE---------------
        // initialize the motors and servos
        initMotorsAndServos();

        // initialize servo position
        grabberL.setPosition(0.5);
        grabberR.setPosition(0.5);

        // adds telemetry that the robot has been initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // resets the runtime
        runtime.reset();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //---------------PRESSES PLAY---------------
            // make a copy of the gamepad
            copyGamepad();

            // does the drive motor stuff
            calcMotorPowerFactor();
            updateDriveMotors();

            // updates the grabber
            updateGrabber();

            // updates the slides level
            updateSlidesLvl();

            // do telemetry
            doTelem();
        }
    }

    // initialize the motors and servos
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

    // makes copies of the gamepad
    public void copyGamepad() {
        try {
            // Store the gamepad values from the previous loop iteration in
            // previousGamepad1 to be used in this loop iteration
            previousGamepad1.copy(currentGamepad1);

            // Store the gamepad values from this loop iteration in
            // currentGamepad1 to be used for the entirety of this loop iteration
            currentGamepad1.copy(gamepad1);
        }
        catch (com.qualcomm.robotcore.exception.RobotCoreException e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1 are being copied from valid Gamepads
        }
    }

    // calculates/updates motorPowerFactor
    public void calcMotorPowerFactor() {
        // rising edge detector for dpad_up; increases motorPowerFactor
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            motorPowerFactor = Math.min(motorPowerFactor + .1, 1);
        }

        // rising edge detector for dpad_down; decreases motorPowerFactor
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            motorPowerFactor = Math.max(motorPowerFactor - .1, 0.1);
        }
    }

    // calculates and updates the power of the drive motors
    public void updateDriveMotors() {
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

        // Send calculated power to wheels
        motorFR.setPower(powerFR);
        motorFL.setPower(powerFL);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    // updates the grabber position
    public void updateGrabber() {
        // rising edge detector for grabber position
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            if (grabberOpen) {
                grabberOpen = false;
                // pickup the cone
                grabberL.setPosition(0.2);
                grabberR.setPosition(0.8);
            } else {
                grabberOpen = true;
                // drop the cone
                grabberL.setPosition(0.5);
                grabberR.setPosition(0.5);
            }
        }
    }

    // move the slides a number of cm
    public void moveSlidesToLvl(int newSlideLvl) {
        int newHeight = (int)(newSlideLvl * 28 * SLIDE_TICS_IN_CM);

        // moves the slides to the desired position
        slideL.setTargetPosition(newHeight);
        slideR.setTargetPosition(newHeight);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setPower(1);
        slideR.setPower(1);

        // set the slideLvl to be what the slides have just moved to
        slideLvl = newSlideLvl;
    }

    // updates the slides level
    public void updateSlidesLvl() {
        // decides power of the slides
        if (currentGamepad1.a && !previousGamepad1.a) {
            moveSlidesToLvl(0);
        } else if (currentGamepad1.b && !previousGamepad1.b) {
            moveSlidesToLvl(1);
        } else if (currentGamepad1.y && !previousGamepad1.y) {
            moveSlidesToLvl(2);
        } else if (currentGamepad1.x && !previousGamepad1.x) {
            moveSlidesToLvl(3);
        }
    }

    // does the telemetry
    public void doTelem() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("motorPowerFactor", motorPowerFactor);
        telemetry.addData("grabberOpen", grabberOpen);
        telemetry.addData("slideLvl", slideLvl);
        telemetry.addData("slideL position", slideL.getCurrentPosition());
        telemetry.addData("slideR position", slideR.getCurrentPosition());
        telemetry.update();
    }
}
