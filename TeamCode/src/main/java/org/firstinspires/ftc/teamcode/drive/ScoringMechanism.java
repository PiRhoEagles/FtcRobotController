package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ScoringMechanism {
    private DcMotor slideL = null;
    private DcMotor slideR = null;
    private Servo grabberL = null;
    private Servo grabberR = null;

    int prevHeight = 0;
    // how many encoder tics make one full slide motor rotation
    static final double SLIDE_TICS_IN_ROT = 384.5;
    // the number of mm the slides move from one motor rotation
    static final double SLIDE_MM_FROM_ROT = 116;
    // number of tics to move slide by 1cm
    static final double SLIDE_TICS_IN_CM = SLIDE_TICS_IN_ROT / (SLIDE_MM_FROM_ROT / 10);


    public void init(HardwareMap hwmap, boolean resetSlideEncoders) {
        slideL = hwmap.get(DcMotor.class, "slideL");
        slideR = hwmap.get(DcMotor.class, "slideR");
        slideR.setDirection(DcMotor.Direction.REVERSE);

        if (resetSlideEncoders) {
            slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        grabberL = hwmap.get(Servo.class, "grabberL");
        grabberR = hwmap.get(Servo.class, "grabberR");

        grabberL.getController().pwmEnable();
        grabberR.getController().pwmEnable();

    }

    public void closeClaw() {
        grabberL.setPosition(0.285); // smaller = closed
        grabberR.setPosition(0.757); // larger = closed
    }

    public void openClaw() {
        grabberL.setPosition(0.41); // smaller = closed
        grabberR.setPosition(0.631); // larger = closed
    }

    public void moveSlides(slidePositions position) {
        int newHeight = 0;

        switch (position) {
            case FLOOR:
                break;
            case LOW:
                newHeight = (int)(36 * SLIDE_TICS_IN_CM);
                break;
            case MID:
                newHeight = (int)(62 * SLIDE_TICS_IN_CM);
                break;
            case HIGH:
                newHeight = (int)(88 * SLIDE_TICS_IN_CM);
                break;
            case STACK2:
                newHeight = (int)(3.25 * SLIDE_TICS_IN_CM);
                break;
            case STACK3:
                newHeight = (int)((3.25 * 2) * SLIDE_TICS_IN_CM);
                break;
            case STACK4:
                newHeight = (int)((3.25 * 3) * SLIDE_TICS_IN_CM);
                break;
            case STACK5:
                newHeight = (int)((3.25 * 4) * SLIDE_TICS_IN_CM);
                break;
        }

        // moves the slides to the desired position
        slideL.setTargetPosition(newHeight);
        slideR.setTargetPosition(newHeight);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (newHeight < prevHeight) {
            // if the slides are moving downward...
            slideL.setPower(0.7);
            slideR.setPower(0.7);
        } else {
            // if the slides are moving upward or are the same level...
            slideL.setPower(1);
            slideR.setPower(1);
        }

        // set the slideLvl to be what the slides have just moved to
        prevHeight = newHeight;
    }

    public void update() {

    }

    public enum slidePositions {
        FLOOR,
        LOW,
        MID,
        HIGH,
        STACK5,
        STACK4,
        STACK3,
        STACK2
    }
}
