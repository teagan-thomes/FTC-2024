package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketFlapAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketScoreAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSpitAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSuckAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide.hSlideToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperDownForTimeAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperStopAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist.WristDownAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.wrist.WristUpAction;
import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.MainDrive;
import org.firstinspires.ftc.teamcode.Components.ViperSlide;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

import java.util.Arrays;

@Autonomous(name="BlueLeft", group="Auto")
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // todo when we add huskyLens
        // Initializing huskyLens
        int visionOutputPosition = 1;

        // Initializing our classes
        ViperSlide viperSlide = new ViperSlide(this);
        MainDrive mainDrive = new MainDrive(this);
        HorizontalSlide hSlide = new HorizontalSlide(this, 3, mainDrive);
        Intake intake = new Intake(this, hSlide, mainDrive);

        viperSlide.resetEncoders();

        // RR-specific initialization
        Pose2d initialPose = new Pose2d(0, 4, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Set velocity and accel constraints
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-25.0, 40);


        VelConstraint slowVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint slowAccelConstraint = new ProfileAccelConstraint(-10, 20);

        double scorexPos = 10.1; //10.3
        double scoreyPos = 16.8;   //17
        double pickUpxPos1 = 14.5; //14.5
        double pickUpxPos2 = 19; //18

        //driveToScorePre
        TrajectoryActionBuilder traj2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToPickUpFirst
        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(pickUpxPos1, 13, Math.toRadians(0)), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 13), Math.toRadians(0),
                        slowVelConstraint,
                        slowAccelConstraint);

        //driveToScoreFirst
        TrajectoryActionBuilder traj5 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToPickUpSecond
        TrajectoryActionBuilder traj7 = traj5.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(pickUpxPos1, 24.5, Math.toRadians(0)), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .strafeToLinearHeading(new Vector2d(pickUpxPos2, 24.5), Math.toRadians(0),
                        slowVelConstraint,
                        slowAccelConstraint);

        //driveToScoreSecond
        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToPickUpThird
        TrajectoryActionBuilder traj9 = traj8.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d((pickUpxPos1 + 3), 16, Math.toRadians(42.5)), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint)
                .strafeToLinearHeading(new Vector2d((pickUpxPos1 + 3 + 5), 22), Math.toRadians(42.5),
                        slowVelConstraint,
                        slowAccelConstraint);

        //driveToScoreThird
        TrajectoryActionBuilder traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(scorexPos, scoreyPos), Math.toRadians(-45),
                        baseVelConstraint,
                        baseAccelConstraint);

        //driveToHang
        TrajectoryActionBuilder traj11 = traj10.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(30, 15, Math.toRadians(90)), Math.toRadians(0),
                        baseVelConstraint,
                        baseAccelConstraint);

        //Actions
        ViperToPositionAction viperAllTheWayUpAtTheTopToScoreASampleInHighBucket = new ViperToPositionAction(viperSlide, 4250);
        ViperDownForTimeAction viperDownALittle = new ViperDownForTimeAction(viperSlide, 300);
        ViperToRestAction viperToRest = new ViperToRestAction(viperSlide);
        ViperStopAction viperStop = new ViperStopAction(viperSlide);

        BucketFlapAction bucketOpen = new BucketFlapAction(viperSlide, "open", 800);
        BucketFlapAction bucketClose = new BucketFlapAction(viperSlide, "close");
        BucketScoreAction bucketScore = new BucketScoreAction(viperSlide, 1000);
        BucketRestAction bucketRest = new BucketRestAction(viperSlide, 500);

        hSlideToPositionAction hSlideForward = new hSlideToPositionAction(hSlide, .57,.53);
        hSlideToPositionAction hSlideBackward = new hSlideToPositionAction(hSlide, .9,.94);

        WristDownAction wristDown = new WristDownAction(intake);
        WristUpAction wristUp = new WristUpAction(intake);

        GrabberSuckAction grabberSuck = new GrabberSuckAction(intake, 2300);
        GrabberSuckAction grabberSuckLonger = new GrabberSuckAction(intake, 3000);
        GrabberSpitAction grabberSpit = new GrabberSpitAction(intake, 2000, 1000);


        // Between initialization and start
        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested() || gamepad1.b) return;

        Action driveToScorePre = traj2.build();
        Action driveToPickUpFirst = traj3.build();
        Action driveToScoreFirst = traj5.build();
        Action driveToPickUpSecond = traj7.build();
        Action driveToScoreSecond = traj8.build();
        Action driveToPickUpThird = traj9.build();
        Action driveToScoreThird = traj10.build();
        Action driveToHang1 = traj11.build();

        // Start
        telemetry.addData("Status", "Running");
        telemetry.update();

        Actions.runBlocking(
                new SequentialAction(

                        //score preload
                        new ParallelAction(
                                driveToScorePre,
                                viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                                bucketClose,
                                bucketScore,
                                hSlideForward,
                                wristDown
                                ),

                        viperStop,
                        bucketOpen,
                        viperDownALittle,

                        //pick up +1
                        new ParallelAction(
                                driveToPickUpFirst,
                                viperToRest,
                                bucketRest,
                                grabberSuck
                        ),

                        //drive to score +1
                        new ParallelAction(
                                driveToScoreFirst,
                                wristUp,
                                hSlideBackward,
                                grabberSpit
                        )
                )
        );

        viperSlide.resetEncoders();

        Actions.runBlocking(
                new SequentialAction(
                        //score +1
                        new ParallelAction(
                                viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                                bucketClose,
                                bucketScore,
                                hSlideForward,
                                wristDown
                                ),

                        viperStop,
                        bucketOpen,
                        viperDownALittle,

                        //pick up +2
                        new ParallelAction(
                                driveToPickUpSecond,
                                viperToRest,
                                bucketRest,
                                grabberSuck
                        ),

                        //drive to score +2
                        new ParallelAction(
                                driveToScoreSecond,
                                wristUp,
                                hSlideBackward,
                                grabberSpit
                        )
                )
        );

        viperSlide.resetEncoders();

        Actions.runBlocking(
                new SequentialAction(
                        //score +2
                        new ParallelAction(
                                viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                                bucketClose,
                                bucketScore,
                                hSlideForward,
                                wristDown
                                ),

                        viperStop,
                        bucketOpen,
                        viperDownALittle,

                        //pick up +3
                        new ParallelAction(
                                driveToPickUpThird,
                                viperToRest,
                                bucketRest,
                                grabberSuckLonger
                        ),

                        //drive to score +3
                        new ParallelAction(
                                driveToScoreThird,
                                wristUp,
                                hSlideBackward,
                                grabberSpit
                        )
                )
        );

        viperSlide.resetEncoders();

        Actions.runBlocking(
                new SequentialAction(
                        //score +3
                        new ParallelAction(
                                viperAllTheWayUpAtTheTopToScoreASampleInHighBucket,
                                bucketClose,
                                bucketScore
                        ),

                        viperStop,
                        bucketOpen,
                        viperDownALittle,

                        //drive to ascent
                        new ParallelAction(
                                driveToHang1,
                                viperToRest,
                                bucketRest
                        )
                )
        );


    }
}
