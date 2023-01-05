package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class AutoTesting extends BaseAuto{

    public static int autoToRun = 2;

    public static int x1 = 50;
    public static int y1 = 0;
    public static int h1 = -82;

    public static int x2 = 50;
    public static int y2 = 3;
    public static int h2 = 45;


    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (autoToRun) {
            case 1: // the main auto
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(52,0,Math.toRadians(0))) // move forward and push signal cone out of way
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // slide high
                        .lineToLinearHeading(new Pose2d(50,14,Math.toRadians(0))) // move left to pole
                        .lineToLinearHeading(new Pose2d(55,14,Math.toRadians(0))) // move forward to pole
                        .addTemporalMarker(() -> mechanism.openClaw()) // open claw
                        .waitSeconds(0.1)
                        .lineToLinearHeading(new Pose2d(48,14,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // slide floor
                        .lineToLinearHeading(new Pose2d(48,-8,Math.toRadians(-90))) // move toward cone stack and turn
                        .lineToLinearHeading(new Pose2d(50,-20,Math.toRadians(-90))) // move to cone stack
                        .build();
            case 2:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little
                        .lineToSplineHeading(new Pose2d(43,0,Math.toRadians(0))) // move near pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // start raising slides to high

                        .lineToSplineHeading(new Pose2d(55,5,Math.toRadians(45))) // turn to face pole and place cone over it
                        .waitSeconds(0.5) // wait for slides to raise
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.5) // wait for grabber to open
                        .lineToSplineHeading(new Pose2d(51,4,Math.toRadians(45))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower slides
                        .lineToSplineHeading(new Pose2d(50,-5,Math.toRadians(-82))) // move towards and face cone stack
                        .lineToSplineHeading(new Pose2d(53,-26,Math.toRadians(-82))) // move to cone stack
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.5) // wait for claw to close
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole

                        .lineToSplineHeading(new Pose2d(55,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.5) // wait for slides to raise
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.5) // wait for grabber to open
                        .lineToSplineHeading(new Pose2d(51,4,Math.toRadians(45))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower slides
                        .lineToSplineHeading(new Pose2d(50,-5,Math.toRadians(-82))) // move towards and face cone stack
                        .lineToSplineHeading(new Pose2d(53,-26,Math.toRadians(-82))) // move to cone stack
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.5) // wait for claw to close
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole

                        .lineToSplineHeading(new Pose2d(55,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.5) // wait for slides to raise
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.5) // wait for grabber to open
                        .lineToSplineHeading(new Pose2d(51,1,Math.toRadians(45))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // lower slides

                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(82))) // move to position 2

                        .build();
            case 3: // turn test
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .turn(Math.toRadians(90))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-90))
                        .build();
        }

        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
