package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestAuto extends BaseAuto{


    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (detectionState) {
            case ONE:
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
            case TWO:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .turn(Math.toRadians(90))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-90))
                        .waitSeconds(2)
                        .turn(Math.toRadians(-90))
                        .build();
            case THREE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.1)
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToLinearHeading(new Pose2d(20,20,Math.toRadians(90)))
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR))
                        .waitSeconds(1)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(0,0,Math.toRadians(0)))
                        .build();
        }

        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
