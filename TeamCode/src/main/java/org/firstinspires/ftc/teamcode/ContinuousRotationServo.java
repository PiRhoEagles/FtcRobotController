package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Continuous Servo Test", group="Linear Opmode")
public class ContinuousRotationServo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo grabberL = null;
    private Servo grabberR = null;

    @Override
    public void runOpMode() {
        grabberL = hardwareMap.get(Servo.class, "grabberL");
        grabberR = hardwareMap.get(Servo.class, "grabberR");

        grabberL.getController().pwmEnable();
        grabberR.getController().pwmEnable();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            grabberL.setPosition(-10);
            grabberR.setPosition(10);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
