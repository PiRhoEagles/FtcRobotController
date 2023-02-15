package org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay.BaseAuto;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
@Disabled
public class PowerPlayRigAuto extends BaseAuto {

    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (detectionState) {
            case ONE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // start raising slides to high
                        .lineToSplineHeading(new Pose2d(55.5,10,Math.toRadians(30))) // move over pole (was 54, 8)
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower slides
                        .splineToLinearHeading(new Pose2d(52, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(54.5,7,Math.toRadians(45))) // turn to pole and place cone over it (was 53.5, 5)
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower slides
                        .splineToLinearHeading(new Pose2d(51, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 3
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 3
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(53.5,7,Math.toRadians(45))) // turn to pole and place cone over it (was 52.5, 5)
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----PARK-----
                        .lineToSplineHeading(new Pose2d(46,0,Math.toRadians(0))) // move to position 2
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // move slides to floor
                        .strafeLeft(23.5) // move to position 1

                        .build();
            case TWO:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // start raising slides to high
                        .lineToSplineHeading(new Pose2d(54,8,Math.toRadians(30))) // move over pole
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower slides
                        .splineToLinearHeading(new Pose2d(52, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(53.5,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower slides
                        .splineToLinearHeading(new Pose2d(52, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 3
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 3
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(52.5,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----PARK-----
                        .lineToSplineHeading(new Pose2d(46,0,Math.toRadians(0))) // move to position 2
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // move slides to floor

                        .build();
            case THREE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // start raising slides to high
                        .lineToSplineHeading(new Pose2d(54,8,Math.toRadians(30))) // move over pole
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower slides
                        .splineToLinearHeading(new Pose2d(52, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(53.5,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(40,0,Math.toRadians(0))) // move back from pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower slides
                        .splineToLinearHeading(new Pose2d(52, -24,Math.toRadians(-82)),Math.toRadians(-40)) // move to cone stack

                        // grab cone 3
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise the slides above the cone stack
                        .lineToSplineHeading(new Pose2d(50,0,Math.toRadians(-82))) // move away from cone stack

                        // move to pole and drop cone 3
                        .lineToSplineHeading(new Pose2d(50,3,Math.toRadians(45))) // move to face pole
                        .lineToSplineHeading(new Pose2d(52.5,5,Math.toRadians(45))) // turn to pole and place cone over it
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----PARK-----
                        .lineToSplineHeading(new Pose2d(46,0,Math.toRadians(0))) // move to position 2
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // move slides to floor
                        .strafeRight(23.5) // move to position 1

                        .build();
        }

        // won't be reached
        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
