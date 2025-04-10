package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperDownForTimeAction;
import org.firstinspires.ftc.teamcode.Roadrunner.Actions.viper.ViperToPositionAction;

import java.util.LinkedList;
import java.util.Objects;
import java.util.Queue;

public class ViperSlide {
    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private Servo leftBucket;
    private Servo rightBucket;
    private Servo bucketFlap;
    private Servo leftSpecimen;
    private Servo rightSpecimen;
    private ElapsedTime bucketCooldownTimer;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    double minFlipLimit = 300;
    int autoFlipPosition = 2800;
    public double holdPower = .2;
    boolean wasScorePressed = false;

    private boolean slideMovedLastIteration = false;
    private double lastStopPosition;

    private Queue<Double> pastPositions;
    private static final int positionHistorySize = 5;
    String specimenGrabberPos = "closed";
    String bucketOpenClose = "open";

    ViperToPositionAction specimenAction = new ViperToPositionAction(this, 2600);
    ViperDownForTimeAction scoreAction = new ViperDownForTimeAction(this, 500);

    public ViperSlide(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
        rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");

        leftBucket = hardwareMap.get(Servo.class, "leftBucket");
        rightBucket = hardwareMap.get(Servo.class, "rightBucket");
        bucketFlap = hardwareMap.get(Servo.class, "bucketFlapServo");

        leftSpecimen = hardwareMap.get(Servo.class, "leftSpecimen");
        rightSpecimen = hardwareMap.get(Servo.class, "rightSpecimen");

        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        this.lastStopPosition = getPos();

        pastPositions = new LinkedList<>();

        bucketRest();
        openBucket();
        grabSpecimen();
    }

    public void setPower(double power) {
        leftViper.setPower(-power);
        rightViper.setPower(power);
    }

    public void checkInputs(
            Float retractTrigger,
            Float extendTrigger,
            Boolean resetEncoders,
            Boolean bucketRest,
            Boolean bucketScore,
            Boolean openBucket,
            Boolean closeBucket,
            Boolean grabSpecimen,
            Boolean releaseSpecimen,
            Boolean bucketSpecimen,
            Boolean bucketSpecimenReset
    ) {
        double retractSpeed = retractTrigger;
        double extendSpeed = extendTrigger;

        if (retractSpeed != 0) {
            leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            setPower(-retractSpeed);
            slideMovedLastIteration = true;
        } else if (extendSpeed != 0) {
            leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            if (getPos() > autoFlipPosition && getPos() < autoFlipPosition + 500) {
                bucketScore();
                closeBucket();
            }
            setPower(extendSpeed);
            slideMovedLastIteration = true;
        } else {
            stop();
            slideMovedLastIteration = false;
        }

        if (slideMovedLastIteration) {
            lastStopPosition = getPos();
        }

        telemetry.addData("last pos: " + lastStopPosition, "current pos: " + getPos());
        telemetry.addData("Left Viper Position: ", leftViper.getCurrentPosition());
        telemetry.addData("Right Viper Position: ", rightViper.getCurrentPosition());
        telemetry.addData("Viper power: ", rightViper.getCurrent(CurrentUnit.AMPS));

        if (resetEncoders) resetEncoders();

        if (bucketScore && getPos() > minFlipLimit) {
            if (Objects.equals(specimenGrabberPos, "open")) grabSpecimen();
            else {
                telemetry.addData("grabber pos", "was closed");
                closeBucket();
                bucketScore();
            }
        }

        if (bucketRest && getPos() > 300 && Objects.equals(specimenGrabberPos, "closed")) {
            bucketRest();
            openBucket();
        }

        if (openBucket) {
            openBucket();
            bucketOpenClose = "open";
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
        } else if (closeBucket) {
            closeBucket();
            bucketOpenClose = "closed";
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
        }

        if (grabSpecimen) grabSpecimen();
        else if (releaseSpecimen) releaseSpecimen();

        if (bucketSpecimen) {
            bucketSpecimen();
            closeBucket();
        }

        if (bucketSpecimenReset) {
            bucketRest();
            openBucket();
        }
    }

    public void stop() {
        lastStopPosition = getPos();
        leftViper.setTargetPosition(-(int) lastStopPosition);
        rightViper.setTargetPosition((int) lastStopPosition);
        leftViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightViper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        telemetry.addData("set target pos to", lastStopPosition);
    }

    public double getPos() {
        return rightViper.getCurrentPosition();
    }

    public void resetEncoders() {
        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void goToPosition(int position) {
        while (rightViper.getCurrentPosition() < position) setPower(1);
        stop();
    }

    public void goToRest() {
        while (rightViper.getCurrentPosition() > 200 && rightViper.getCurrent(CurrentUnit.AMPS) < 6) setPower(-1);
        stop();
    }

    public Action goToPositionAction(int position) {
        int error = 50;
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (getPos() < position) setPower(1);
                else if (getPos() > position) setPower(-1);
                else stop();
                packet.put("Viper Position", getPos());
                return getPos() >= (position - error) && getPos() <= (position + error);
            }
        };
    }

    public double getCurrentPower() {
        return rightViper.getCurrent(CurrentUnit.AMPS);
    }

    public void setBucketPosition(double leftPosition, double rightPosition) {
        leftBucket.setPosition(leftPosition);
        rightBucket.setPosition(rightPosition);
    }

    public void setBucketFlapPosition(double position) {
        bucketFlap.setPosition(position);
        telemetry.addData("Bucket Flap Position", position);
    }

    public void bucketRest() {
        setBucketPosition(0, 1);
    }

    public void bucketScore() {
        setBucketPosition(1, 0);
    }

    public void bucketSpecimen() {
        setBucketPosition(.2, .8);
    }

    public void closeBucket() {
        setBucketFlapPosition(.55);
    }

    public void openBucket() {
        setBucketFlapPosition(.4);
    }

    public void setSpecimenGrabberPos(double position) {
    }

    public void grabSpecimen() {
        leftSpecimen.setPosition(0);
        rightSpecimen.setPosition(.7);
        specimenGrabberPos = "closed";
    }

    public void releaseSpecimen() {
        leftSpecimen.setPosition(.6);
        rightSpecimen.setPosition(.4);
        specimenGrabberPos = "open";
    }
}
