package org.firstinspires.ftc.teamcode.Components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Timer;

public class ViperSlide {
    private DcMotorEx leftViper;
    private DcMotorEx rightViper;

    private Servo leftBucket;
    private Servo rightBucket;
    private Servo bucketFlap;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    double minFlipLimit = 650;
    public double holdPower = .2;

    private ElapsedTime bucketCooldownTimer;

    double lastPosition;




    public ViperSlide(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

        leftViper = hardwareMap.get(DcMotorEx.class, "leftViper");
        rightViper = hardwareMap.get(DcMotorEx.class, "rightViper");

        leftBucket = hardwareMap.get(Servo.class, "leftBucket");
        rightBucket = hardwareMap.get(Servo.class, "rightBucket");
        bucketFlap = hardwareMap.get(Servo.class, "bucketFlapServo");

        leftViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightViper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightViper.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // initialize
        leftBucket.setPosition(1);
        rightBucket.setPosition(0);
    }

    public void setPower(double power) {
        leftViper.setPower(-power);
        rightViper.setPower(power);
    }

    public void checkInputs(
            Float retractSpeed,
            Float extendSpeed,
            Boolean resetEncoders,
            Boolean hold,
            Boolean bucketRest,
            Boolean bucketScore,
            Boolean openBucket,
            Boolean closeBucket,
            Boolean grabSpecimen,
            Boolean releaseSpecimen
    ) {
        // Move Viper
        if (retractSpeed != 0) {
            setPower(-retractSpeed);
        }
        else if (extendSpeed != 0) {
            setPower(extendSpeed);
        }
        else {
            stop();
        }
        telemetry.addData("Viper Position: ", rightViper.getCurrentPosition());
        telemetry.addData("Viper power: ", rightViper.getCurrent(CurrentUnit.AMPS));

        // Hold Position
        if(hold) {
            holdPosition(getPos());
        }

        // Reset Encoders
        if(resetEncoders) {
            resetEncoders();
        }

        // Bucket
        if(bucketScore) {
            if(getPos() > minFlipLimit) {
                bucketScore();
            }
            else {
                bucketCooldownTimer = new ElapsedTime();
            }
        }

        if(bucketRest) {
            bucketRest();
//            if(bucketCooldownTimer != null && bucketCooldownTimer.seconds() < 1)
//                bucketRest();
//            else
//                bucketCooldownTimer = null;
        }

        if(openBucket) {
            openBucket();
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
            telemetry.addData("Bucket Flap Position", "open");
        }
        else if(closeBucket) {
            closeBucket();
            telemetry.addData("Bucket Flap Position", bucketFlap.getPosition());
            telemetry.addData("Bucket Flap Position", "closed");
        }

        if(grabSpecimen) {
            // grab specimen
        }
        else if(releaseSpecimen) {
            // release specimen
        }
    }

    public void stop() {
        leftViper.setPower(0);
        rightViper.setPower(0);
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

    public void holdPosition(double holdPosition) {
        double holdPower = 0.1;
        double holdPowerIncrement = 0.2;


        telemetry.addData("Holding", holdPosition);

        if(lastPosition < getPos()) {
            telemetry.addData("dropped " + (getPos() - lastPosition), " since last update");
        }
        else if(lastPosition > getPos()) {
            telemetry.addData("raised " + (lastPosition - getPos()), " since last update");
        }
        else {
            telemetry.addData("not moving", "since last update");
        }

        lastPosition = getPos();


        if(rightViper.getCurrentPosition() > holdPosition) {
            holdPower = -holdPowerIncrement;
            leftViper.setPower(-holdPower);
            rightViper.setPower(holdPower);
            telemetry.addData("holdPower", holdPower);
        } else if(rightViper.getCurrentPosition() < holdPosition) {
            holdPower = -holdPowerIncrement;
            leftViper.setPower(holdPower);
            rightViper.setPower(holdPower);
            telemetry.addData("holdPower", holdPower);
        } else {
            stop();
        }

        telemetry.addData("Viper Position: ", rightViper.getCurrentPosition());

    }

    public void goToPosition(int position) {
        while (rightViper.getCurrentPosition() < position) {
            setPower(1);
        }
        stop();
    }

    public void goToRest() {
        while ((rightViper.getCurrentPosition() > 200) && (rightViper.getCurrent(CurrentUnit.AMPS) < 6)) {
            setPower(-1);
        }
        stop();
    }

    public Action goToPositionAction(int position) {
        int error = 50;
        return new Action() {
            public boolean run(@NonNull TelemetryPacket packet) {
                if (getPos() < position) {
                    setPower(1);
                }
                else if(getPos() > position) {
                    setPower(-1);
                }
                else {
                    stop();
                }

                packet.put("Viper Position", getPos());

                return getPos() >= (position - error) && getPos() <= (position + error);
            }
        };
    }


    // bucket


    public void setBucketPosition(double leftPosition, double rightPosition) {
        leftBucket.setPosition(leftPosition);
        rightBucket.setPosition(rightPosition);
    }

    public void setBucketFlapPosition(double position) {
        bucketFlap.setPosition(position);
        telemetry.addData("Bucket Flap Position", position);
    }

    public void bucketRest() {
        setBucketPosition(1, 0);
    }

    public void bucketScore() {
        setBucketPosition(.49, .51);
    }

    public void openBucket() {
        setBucketFlapPosition(1);
    }

    public void closeBucket() {
        setBucketFlapPosition(.4 ); // .75
    }


}