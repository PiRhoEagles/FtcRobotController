package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Power Play Mecanum Drive", group="Linear Opmode")
//@Disabled
public class PowerPlayMecanumDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFR = null;
    private DcMotor motorFL = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        motorFR = hardwareMap.get(DcMotor.class, "FR");
        motorFL = hardwareMap.get(DcMotor.class, "FL");
        motorBL = hardwareMap.get(DcMotor.class, "BL");
        motorBR = hardwareMap.get(DcMotor.class, " BR");

        // Most robots need the motors on one side to be reversed to drive forward
        // Reverse the motors that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // the power of the motors are multiplied by this
        double motorPowerFactor = 0.6;

        // copies of the gamepad
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            //----------CALC MOTOR POWER----------

            // joystick values used to determint drive movement
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


            //----------SET MOTOR POWER----------

            // Send calculated power to wheels
            motorFR.setPower(powerFR);
            motorFL.setPower(powerFL);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);


            //----------TELEMETRY----------

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("motorPowerFactor", motorPowerFactor);

            telemetry.addData("LeftX", gamepad1.left_stick_x);
            telemetry.addData("LeftY", -gamepad1.left_stick_y);
            telemetry.addData("RightX", gamepad1.right_stick_x);

            telemetry.addData("FR ", powerFR);
            telemetry.addData("FL", powerFL);
            telemetry.addData("BL", powerBL);
            telemetry.addData("BR", powerBR);

            telemetry.update();
        }
    }
}
