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

    public static double poleX = 57; // 56
    public static double poleY = 7; // 6
    public static double poleH = 50;

    public static double backFromPoleX = 38;
    public static double backFromPoleY = 3.5;
    public static double backFromPoleH = 0;

    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (detectionState) {
            case ONE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little

                        // go to pole and place cone
                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise high
                        .lineToSplineHeading(new Pose2d(poleX,poleY,Math.toRadians(poleH)))
                        .waitSeconds(0.3) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.2) // wait for grabber to open

                        // -----CYCLE 1-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 2-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY-2,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 3-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK3)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY-3,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 4-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY-4,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw()) // close claw and grab cone
                        .waitSeconds(0.3) // wait for claw to close

                        // move to pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.LOW))
                        .lineToSplineHeading(new Pose2d(backFromCSX,backFromCSY,Math.toRadians(backFromCSH)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(poleX,poleY-3,Math.toRadians(poleH)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.3) // wait for grabber to open

                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // lower

                        // move left to position 1
                        .lineToSplineHeading(new Pose2d(45,24,Math.toRadians(0)))

                        .build();
            case TWO:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little

                        // go to pole and place cone
                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise high
                        .lineToSplineHeading(new Pose2d(poleX,poleY,Math.toRadians(poleH)))
                        .waitSeconds(0.3) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.2) // wait for grabber to open

                        // -----CYCLE 1-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 2-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 3-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK3)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 4-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // lower

                        .build();
            case THREE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // raise cone a little

                        // go to pole and place cone
                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH)) // raise high
                        .lineToSplineHeading(new Pose2d(poleX,poleY,Math.toRadians(poleH)))
                        .waitSeconds(0.3) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw()) // drops cone on pole
                        .waitSeconds(0.2) // wait for grabber to open

                        // -----CYCLE 1-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 2-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 3-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK3)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        // -----CYCLE 4-----
                        // go to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPoleX,backFromPoleY,Math.toRadians(backFromPoleH))) // move back
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK2)) // lower
                        .lineToSplineHeading(new Pose2d(coneStackX,coneStackY,Math.toRadians(coneStackH))) // to cone stack

                        // grab cone
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

                        .lineToSplineHeading(new Pose2d(45,0,Math.toRadians(0))) // to center area
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // lower

                        // move left to position 3
                        .lineToSplineHeading(new Pose2d(45,-24,Math.toRadians(0)))

                        .build();
        }

        // won't be reached
        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
