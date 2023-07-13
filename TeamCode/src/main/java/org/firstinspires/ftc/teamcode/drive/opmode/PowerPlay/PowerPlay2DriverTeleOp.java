package org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay;

import static org.firstinspires.ftc.teamcode.drive.ScoringMechanism.slidePositions.FLOOR;
import static org.firstinspires.ftc.teamcode.drive.ScoringMechanism.slidePositions.LOW;
import static org.firstinspires.ftc.teamcode.drive.ScoringMechanism.slidePositions.MID;
import static org.firstinspires.ftc.teamcode.drive.ScoringMechanism.slidePositions.HIGH;
import static org.firstinspires.ftc.teamcode.drive.ScoringMechanism.slidePositions.STACK2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;

import java.text.DecimalFormat;


@TeleOp(name="PP: 2 Driver TeleOp", group="Power Play")
public class PowerPlay2DriverTeleOp extends LinearOpMode { //---------------OPEN OPMODE---------------

    private ElapsedTime timeSinceClosedGrabber = new ElapsedTime();
    private boolean grabberWasRaisedAfterClosing = true;

    // Setup scoring mechanism.
    protected ScoringMechanism mechanism = new ScoringMechanism();

    // Setup drive motors.
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;

    // The power of the motors are multiplied by this.
    double motorPowerFactor = 1.0;

    // Decimal format that rounds to one decimal place.
    DecimalFormat df = new DecimalFormat("#.#");

    // Copies of the gamepads.
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void runOpMode() { //---------------PRESS INIT---------------

        initHardware();
        waitForStart();

        // runs until the end of the match (driver presses STOP)
        while (opModeIsActive()) { //---------------PRESSES PLAY---------------
            copyGamepads();

            calcMotorPowerFactor();

            updateDriveMotors();
            updateGrabber();
            updateSlidesLvl();

            doTelem();
        }
    }


    // Initializes the hardware.
    public void initHardware() {
        mechanism.init(hardwareMap, false);

        // Setup drive motors.
        motorFR = hardwareMap.get(DcMotor .class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, "BR");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Copy the Gamepads.
    public void copyGamepads() {
        try {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
        }
        catch (Exception e) {
            // Swallow the possible exception, it should not happen as
            // currentGamepad1 are being copied from valid Gamepads
        }
    }

    // Calculates/updates motorPowerFactor.
    public void calcMotorPowerFactor() {
        // Rising edge detector for dpad_up on gp1; increases motorPowerFactor
        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
            motorPowerFactor = Math.min(motorPowerFactor + .1, 1);
            df.format(motorPowerFactor);
        }

        // Rising edge detector for dpad_down on gp1; decreases motorPowerFactor
        if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
            motorPowerFactor = Math.max(motorPowerFactor - .1, 0.1);
            df.format(motorPowerFactor);
        }
    }

    // Calculates and updates the power of the drive motors.
    public void updateDriveMotors() {
        // gamepad inputs
        double lsy = -gamepad1.left_stick_y; // Remember, this is reversed!
        double lsx = gamepad1.left_stick_x;
        double rsx = gamepad1.right_stick_x;

        // disregards lsy or lsx if their absolute value is less than 0.1
        if (Math.abs(lsy) < 0.1) {
            lsy = 0.0;
        }
        if (Math.abs(lsx) < 0.1) {
            lsx = 0.0;
        }

        // gets the signs of the stick values
        double lsySign = lsy / Math.abs(lsy);
        double lsxSign = lsx / Math.abs(lsx);
        double rsxSign = rsx / Math.abs(rsx);

        // ensures the stick value signs aren't NaN
        if (Double.isNaN(lsySign)) {lsySign = 0;}
        if (Double.isNaN(lsxSign)) {lsxSign = 0;}
        if (Double.isNaN(rsxSign)) {rsxSign = 0;}

        // joystick values used to determine drive movement
        // they're squared to allow for finer control at low speeds
        double y = Math.pow(lsy, 2) * lsySign;
        double x = Math.pow(lsx, 2) * lsxSign;
        double rx = Math.pow(rsx, 2) * rsxSign;

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

    // Updates the grabber position.
    public void updateGrabber() {
        // Rising edge detector for right bumper on gp2.
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
            mechanism.closeClaw();
            timeSinceClosedGrabber.reset();
            grabberWasRaisedAfterClosing = false;
        }

        // Rising edge detector for left bumper on gp2.
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
            mechanism.openClaw();
        }

        if (timeSinceClosedGrabber.milliseconds() > 250 && !grabberWasRaisedAfterClosing) {
            mechanism.moveSlides(STACK2);
            grabberWasRaisedAfterClosing = true;
        }
    }

    // Updates the slides level.
    public void updateSlidesLvl() {
        if (currentGamepad2.a && !previousGamepad2.a) {
            mechanism.moveSlides(FLOOR);
        } else if (currentGamepad2.b && !previousGamepad2.b) {
            mechanism.moveSlides(LOW);
        } else if (currentGamepad2.y && !previousGamepad2.y) {
            mechanism.moveSlides(MID);
        } else if (currentGamepad2.x && !previousGamepad2.x) {
            mechanism.moveSlides(HIGH);
        } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
            mechanism.moveSlides(STACK2);
        }
    }

    // Does the telemetry.
    public void doTelem() {
        telemetry.addData("motorPowerFactor", motorPowerFactor);
        telemetry.update();
    }
}
