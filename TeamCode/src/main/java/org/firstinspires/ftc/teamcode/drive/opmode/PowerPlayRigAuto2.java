package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous
public class PowerPlayRigAuto2 extends BaseAuto{

    public static double coneStackX = 52;
    public static double coneStackY = -24.5;
    public static double coneStackH = -84;

    public static double backFromCSX = 47;
    public static double backFromCSY = -6;
    public static double backFromCSH = -84;

    public static double poleX = 56;
    public static double poleY = 6;
    public static double poleH = 50;


    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (detectionState) {
            case ONE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // open and raise claw
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little

                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(48,0,Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(49,0,Math.toRadians(-82)))
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH)))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move to pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.LOW))
                        .lineToSplineHeading(new Pose2d(backFromCSX,backFromCSY,Math.toRadians(backFromCSH)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(poleX,poleY,Math.toRadians(poleH)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        // move back to cone stack
                        .back(1)
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR))
                        .lineToSplineHeading(new Pose2d(backFromCSX,backFromCSY,Math.toRadians(backFromCSH)))
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH)))


                        .build();
            case TWO: case THREE:
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
        }

        // won't be reached
        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
