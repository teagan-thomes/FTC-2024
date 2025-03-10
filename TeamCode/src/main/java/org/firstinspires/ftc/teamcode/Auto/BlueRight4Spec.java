package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketFlapAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketRestAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketScoreAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.bucket.BucketSpecimenAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSpitAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.grabber.GrabberSuckAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide.hSlideToPositionAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen.SpecimenGrabberGrabAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.specimen.SpecimenGrabberReleaseAction;
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
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@Autonomous(name = "BlueRight4Spec", group = "Auto")
public class BlueRight4Spec extends LinearOpMode {
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

		// hSlide.resetEncoder();
		viperSlide.resetEncoders();

		// RR-specific initialization
		Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-180));
		MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
		drive.setProfile(MecanumDrive.DriveProfile.BASE);

		// Set velocity and accel constraints
		// normal
		VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
				new TranslationalVelConstraint(50),
				new AngularVelConstraint(Math.PI / 2)));
		AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-30, 40);

		// slow for scoring
		VelConstraint scoreVelConstraint = new MinVelConstraint(Arrays.asList(
				new TranslationalVelConstraint(25),
				new AngularVelConstraint(Math.PI / 2)));
		AccelConstraint scoreAccelConstraint = new ProfileAccelConstraint(-30, 30);

		// intake
		VelConstraint intakeVelConstraint = new MinVelConstraint(Arrays.asList(
				new TranslationalVelConstraint(20),
				new AngularVelConstraint(Math.PI / 2)));
		AccelConstraint intakeAccelConstraint = new ProfileAccelConstraint(-10, 20);

		// Trajectories
		// driveToScorePre
		TrajectoryActionBuilder traj1 = drive.actionBuilder(initialPose)
				.strafeTo(new Vector2d(28, 0),
						scoreVelConstraint,
						scoreAccelConstraint);

		// driveToGetFirst
		TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(15, -19), Math.toRadians(-45),
					baseVelConstraint,
					baseAccelConstraint);

		TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(24, -28), Math.toRadians(-45),
						intakeVelConstraint,
						intakeAccelConstraint);

		TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(15, -28), Math.toRadians(-135),
						baseVelConstraint,
						baseAccelConstraint);

		// driveToGetSecond
		TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(15, -28.5), Math.toRadians(-45),
						baseVelConstraint,
						baseAccelConstraint);

		TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(25, -38.5), Math.toRadians(-45),
						intakeVelConstraint,
						intakeAccelConstraint);

		TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(15, -28), Math.toRadians(-135),
						baseVelConstraint,
						baseAccelConstraint);

		// driveToScoreFirst
		TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(-0.5, -28), Math.toRadians(0),
						baseVelConstraint,
						baseAccelConstraint);

		TrajectoryActionBuilder traj9 = traj8.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(30, 3), Math.toRadians(-180),
						baseVelConstraint,
						baseAccelConstraint);

		// driveToScoreSecond
		TrajectoryActionBuilder traj10 = traj9.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(-0.5, -28), Math.toRadians(0),
						baseVelConstraint,
						baseAccelConstraint);

		TrajectoryActionBuilder traj11 = traj10.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(32, 6), Math.toRadians(-180),
						baseVelConstraint,
						baseAccelConstraint);

		// driveToScoreThird
		TrajectoryActionBuilder traj12 = traj11.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(-0.5, -28), Math.toRadians(0),
						baseVelConstraint,
						baseAccelConstraint);

		TrajectoryActionBuilder traj13 = traj12.endTrajectory().fresh()
				.strafeToLinearHeading(new Vector2d(34, 9), Math.toRadians(-180),
						baseVelConstraint,
						baseAccelConstraint);


		// Actions
		ViperToPositionAction viperSpecimen = new ViperToPositionAction(viperSlide, 2600);
		ViperDownForTimeAction viperScore = new ViperDownForTimeAction(viperSlide, 500);
		ViperToRestAction viperToRest = new ViperToRestAction(viperSlide);
		ViperStopAction viperStop = new ViperStopAction(viperSlide);

		BucketSpecimenAction bucketSpecimen = new BucketSpecimenAction(viperSlide, 1000);
		BucketSpecimenAction bucketSpecimenQuick = new BucketSpecimenAction(viperSlide, 500);
		BucketRestAction bucketRest = new BucketRestAction(viperSlide, 0);

		hSlideToPositionAction hSlideForward = new hSlideToPositionAction(hSlide, .19, .16);
		hSlideToPositionAction hSlideForwardLess = new hSlideToPositionAction(hSlide, .7, .74);
		hSlideToPositionAction hSlideForwardClose = new hSlideToPositionAction(hSlide, .44, .41);
		hSlideToPositionAction hSlideBackward = new hSlideToPositionAction(hSlide, .9, .94);
		hSlideToPositionAction hSlideBackwardLess = new hSlideToPositionAction(hSlide, .49, .46);


		WristDownAction wristDown = new WristDownAction(intake);
		WristUpAction wristUp = new WristUpAction(intake);

		GrabberSuckAction grabberSuck = new GrabberSuckAction(intake, 1250);
		GrabberSuckAction grabberSuckLonger = new GrabberSuckAction(intake, 1500);
		GrabberSpitAction grabberSpit = new GrabberSpitAction(intake, 1500, 750);
		GrabberSpitAction grabberSpitLonger = new GrabberSpitAction(intake, 1500, 850);


		SpecimenGrabberGrabAction specimenGrab = new SpecimenGrabberGrabAction(viperSlide, 1500);
		SpecimenGrabberGrabAction specimenGrabLong = new SpecimenGrabberGrabAction(viperSlide, 2500);
		SpecimenGrabberGrabAction specimenGrabLonger = new SpecimenGrabberGrabAction(viperSlide, 2850);
		SpecimenGrabberReleaseAction specimenRelease = new SpecimenGrabberReleaseAction(viperSlide, 75);

		// Between initialization and start
		while (!isStopRequested() && !opModeIsActive()) {
			int position = visionOutputPosition;
			telemetry.addData("Position during Init", position);
			telemetry.addData("Initial Pose", "x: %.2f, y: %.2f, heading: %.2f",
					initialPose.position.x, initialPose.position.y,
					Math.toDegrees(initialPose.heading.toDouble()));
			telemetry.update();
		}

		waitForStart();

		if (isStopRequested() || gamepad1.b)
			return;

		Action driveToScorePre = traj1.build();
		Action driveToGetFirst1 = traj2.build();
		Action driveToGetFirst2 = traj3.build();
		Action driveToGetFirst3 = traj4.build();
		Action driveToGetSecond1 = traj5.build();
		Action driveToGetSecond2 = traj6.build();
		Action driveToGetSecond3 = traj7.build();
		Action driveToScoreFirst1 = traj8.build();
		Action driveToScoreFirst2 = traj9.build();
		Action driveToScoreSecond1 = traj10.build();
		Action driveToScoreSecond2 = traj11.build();
		Action driveToScoreThird1 = traj12.build();
		Action driveToScoreThird2 = traj13.build();

		// Start
		telemetry.addData("Status", "Running");
		telemetry.addData("Current Pose", "x: %.2f, y: %.2f, heading: %.2f",
				drive.pose.position.x, drive.pose.position.y,
				Math.toDegrees(drive.pose.heading.toDouble()));
		telemetry.update();

		// score pre
		telemetry.addData("Executing", "Score Pre");
		telemetry.update();
		Actions.runBlocking(
				new SequentialAction(
						new ParallelAction(
								driveToScorePre,
								viperSpecimen,
								bucketSpecimenQuick),

						new ParallelAction(
								viperScore,
								specimenRelease
						),

						new ParallelAction(
								driveToGetFirst1,
								viperToRest,
								bucketRest,
								hSlideForwardLess,
								wristDown),

						new ParallelAction(
								driveToGetFirst2,
								hSlideForward,
								grabberSuck),

						new ParallelAction(
								driveToGetFirst3,
								hSlideBackwardLess,
								grabberSpit),

						driveToGetSecond1,

						new ParallelAction(
								driveToGetSecond2,
								hSlideForwardClose,
								grabberSuckLonger
						),

						new ParallelAction(
								driveToGetSecond3,
								hSlideBackwardLess,
								grabberSpitLonger
						),

						new ParallelAction(
								driveToScoreFirst1,
								wristUp,
								specimenGrab
						)
				)
		);

		viperSlide.resetEncoders();
		sleep(50);

		Actions.runBlocking(
				new SequentialAction(
						new ParallelAction(
								driveToScoreFirst2,
								hSlideBackward,
								viperSpecimen,
								bucketSpecimen
						),

						new ParallelAction(
								viperScore,
								specimenRelease
						),

						new ParallelAction(
								driveToScoreSecond1,
								viperToRest,
								bucketRest,
								specimenGrabLong
						)
				)
		);

		viperSlide.resetEncoders();
		sleep(50);

		Actions.runBlocking(
				new SequentialAction(
						new ParallelAction(
								driveToScoreSecond2,
								viperSpecimen,
								bucketSpecimen),

						new ParallelAction(
								viperScore,
								specimenRelease
						),

						new ParallelAction(
								driveToScoreThird1,
								viperToRest,
								bucketRest,
								specimenGrabLonger
						)
				)
		);

		viperSlide.resetEncoders();
		sleep(50);

		Actions.runBlocking(
				new SequentialAction(
						new ParallelAction(
								driveToScoreThird2,
								viperSpecimen,
								bucketSpecimen),

						new ParallelAction(
								viperScore,
								specimenRelease
						)
				)
		);

		telemetry.addData("Final Pose", "x: %.2f, y: %.2f, heading: %.2f",
				drive.pose.position.x, drive.pose.position.y,
				Math.toDegrees(drive.pose.heading.toDouble()));
		telemetry.update();

		// viperSlide.resetEncoders();
		//
		// Actions.runBlocking(
		// new SequentialAction(
		//// driveToPushFirst2,
		//// driveToPushFirst3,
		//// driveToPushFirst4,
		//// driveToPushSecond1,
		//// driveToPushSecond2,
		//// driveToPushSecond3,
		// spinAction,
		// driveToPickUpFirst
		// )
		// );

		// viperSlide.grabSpecimen(); //TODO grabSpecimen action doesn't work
		// sleep(500);

		// score first
		// Actions.runBlocking(
		// new ParallelAction(
		// driveToScoreFirst,
		// viperSpecimen,
		// bucketSpecimen
		// )
		// );
		// Actions.runBlocking(
		// new ParallelAction(
		// viperScore,
		// specimenRelease
		// )
		// );
		//
		// // pick up second
		// Actions.runBlocking(
		// new ParallelAction(
		// driveToPickUpSecond,
		// viperToRest,
		// bucketRest
		// )
		// );
		//
		// viperSlide.resetEncoders();
		//
		// Actions.runBlocking(driveToPickUpSecond2);
		//
		// viperSlide.grabSpecimen(); //TODO grabSpecimen action doesn't work
		// sleep(500);
		//
		// // score second
		// Actions.runBlocking(
		// new ParallelAction(
		// driveToScoreSecond,
		// viperSpecimen,
		// bucketSpecimen
		// )
		// );
		// Actions.runBlocking(
		// new ParallelAction(
		// viperScore,
		// specimenRelease
		// )
		// );
		//
		// //park
		// Actions.runBlocking(
		// new ParallelAction(
		// park,
		// viperToRest,
		// bucketRest
		// )
		// );
		//
		// viperSlide.resetEncoders();

	}

}