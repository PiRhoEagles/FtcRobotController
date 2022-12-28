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
}
