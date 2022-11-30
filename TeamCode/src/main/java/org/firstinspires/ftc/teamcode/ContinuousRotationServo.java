package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Continuous Servo Test", group="Linear Opmode")
@Disabled
public class ContinuousRotationServo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo grabberL = null;
    private Servo grabberR = null;

    @Override
    public void runOpMode() {
        // maps the grabber servos
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // enables pwm for the grabber servos
        // make sure this is after the driver presses play, otherwise
        // the servos will begin spinning after init and not play
        grabberL.getController().pwmEnable();
        grabberR.getController().pwmEnable();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // negative makes it spin backwards, positive is forwards
            // not sure what the magnitude of the value does because for any
            // magnitude it always spins the same speed
            grabberL.setPosition(-10);
            grabberR.setPosition(10);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
