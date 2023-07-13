package org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.ScoringMechanism;
import org.firstinspires.ftc.teamcode.drive.opmode.PowerPlay.BaseAuto;
import org.firstinspires.ftc.teamcode.pipelines.PowerPlayPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name="PP: Right Auto", group="Power Play")
public class PowerPlayRigAutoBetter extends BaseAuto {

    public static customPose2D poleFirst = new customPose2D(55, 8, 30);
    public static customPose2D backFromPole = new customPose2D(40, 0, 0);
    public static customPose2D coneStack = new customPose2D(52, -24, -82, -40);
    public static customPose2D backFromCS = new customPose2D(50, 0, -82);
    public static customPose2D facePole = new customPose2D(50, 3, 45);
    public static customPose2D pole = new customPose2D(53, 5, 45);
    
    @Override
    public TrajectorySequence trajectorySequenceBuilder(PowerPlayPipeline.detectionStates detectionState) {
        switch (detectionState) {
            case ONE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        //.addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(poleFirst.x,poleFirst.y,Math.toRadians(poleFirst.h)))
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y-1,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----PARK-----
                        .lineToSplineHeading(new Pose2d(47,0,Math.toRadians(0))) // move to position 2
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // move slides to floor
                        .strafeLeft(23.5) // move to position 1

                        .build();
            case TWO:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        //.addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(poleFirst.x,poleFirst.y,Math.toRadians(poleFirst.h)))
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y-1,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        .build();
            case THREE:
                return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                        // raise cone a little and move towards pole
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(42,0,Math.toRadians(15))) // move near pole

                        // raise slides to high & move to place cone 1 over pole
                        //.addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(poleFirst.x,poleFirst.y,Math.toRadians(poleFirst.h)))
                        .waitSeconds(0.2) // wait for slides to raise

                        // drops cone 1
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 2-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK5))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----CYCLE 3-----
                        // move to cone stack
                        .lineToSplineHeading(new Pose2d(backFromPole.x,backFromPole.y,Math.toRadians(backFromPole.h)))
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.STACK4))
                        .splineToLinearHeading(new Pose2d(coneStack.x, coneStack.y-1,Math.toRadians(coneStack.h)),Math.toRadians(coneStack.tan))

                        // grab cone 2
                        .waitSeconds(0.1) // wait for robot to stop shaking
                        .addTemporalMarker(() -> mechanism.closeClaw())
                        .waitSeconds(0.3) // wait for claw to close

                        // move away from cone stack
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.HIGH))
                        .lineToSplineHeading(new Pose2d(backFromCS.x,backFromCS.y,Math.toRadians(backFromCS.h)))

                        // move to pole and drop cone 2
                        .lineToSplineHeading(new Pose2d(facePole.x,facePole.y,Math.toRadians(facePole.h)))
                        .lineToSplineHeading(new Pose2d(pole.x,pole.y,Math.toRadians(pole.h)))
                        .waitSeconds(0.2) // wait for slides to stop shaking
                        .addTemporalMarker(() -> mechanism.openClaw())
                        .waitSeconds(0.3) // wait for grabber to open

                        //-----PARK-----
                        .lineToSplineHeading(new Pose2d(47,0,Math.toRadians(0))) // move to position 2
                        .addTemporalMarker(() -> mechanism.moveSlides(ScoringMechanism.slidePositions.FLOOR)) // move slides to floor
                        .strafeRight(23.5) // move to position 3

                        .build();
        }

        // won't be reached
        return drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .build();
    }
}
