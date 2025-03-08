package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Objects;

public class HorizontalSlide {
    // private DcMotorEx slideMotor;
    private Intake intake;
    private Servo leftLinkage;
    private Servo rightLinkage;

    private final double power;
    private final double currentLimit;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    double speed = 0.013;
    private ElapsedTime timer;
    private static final double MOVE_BACK_DELAY_MS = 300; // 500ms delay
    private boolean isWaitingToMoveBack = false;

    // int maxSlidePosition = 650; // 675
    // private final Gamepad gamepad1;
    // private final Gamepad gamepad2;

    public HorizontalSlide(OpMode opMode, double currentLimit, MainDrive mainDrive) {
        // slideMotor = opMode.hardwareMap.get(DcMotorEx.class, "slideMotor");
        // slideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftLinkage = opMode.hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = opMode.hardwareMap.get(Servo.class, "rightLinkage");

        this.power = 1;
        this.currentLimit = currentLimit;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.timer = new ElapsedTime();
        this.intake = new Intake(opMode, this, mainDrive);
    }

    public void actionControl(double leftPos, double rightPos) {
        leftLinkage.setPosition(leftPos);
        rightLinkage.setPosition(rightPos);
    }
    public void moveForward() {
        leftLinkage.setPosition(0.60); //.32 is ~flip limit, 0.57
        rightLinkage.setPosition(0.56); //.28 is ~flip limit, 0.53
    }

    public void moveBackward() {
        leftLinkage.setPosition(0.9);
        rightLinkage.setPosition(0.94);
    }

    public void moveForwardGradually() {
        leftLinkage.setPosition(Range.clip((leftLinkage.getPosition() - speed), 0.09, 0.9));
        rightLinkage.setPosition(Range.clip((rightLinkage.getPosition() - speed), 0.06, 0.94));
//        leftLinkage.setPosition(leftLinkage.getPosition() - speed);
//        rightLinkage.setPosition(rightLinkage.getPosition() - speed);

    }

    public void moveBackwardGradually() {
        leftLinkage.setPosition(Range.clip((leftLinkage.getPosition() + speed), 0.09, 0.9));
        rightLinkage.setPosition(Range.clip((rightLinkage.getPosition() + speed), 0.06, 0.94));
//        leftLinkage.setPosition(leftLinkage.getPosition() + speed);
//        rightLinkage.setPosition(rightLinkage.getPosition() + speed);
        ;
    }

    public void checkInputs(double extend, double retract, boolean extendGradual, boolean retractGradual) {
        if (extend > 0.1) {
            moveForward();
            isWaitingToMoveBack = false;
        } else if (retract > 0.1) {
            if (!isWaitingToMoveBack) {
                intake.wristUp();
                timer.reset();
                isWaitingToMoveBack = true;
            }

            if (timer.milliseconds() >= MOVE_BACK_DELAY_MS) {
                moveBackward();
                isWaitingToMoveBack = false;
            }
        } else {
            isWaitingToMoveBack = false;
            if (extendGradual) {
                moveForwardGradually();
            }
            if (retractGradual) {
                moveBackwardGradually();
            }
        }
    }

    public double getPos() {
        return leftLinkage.getPosition();
    }

    // public void moveForward() {
    // if (slideMotor.getCurrentPosition() < maxSlidePosition) {
    // slideMotor.setPower(power);
    // } else {
    // stopMotor();
    // }
    // }
    //
    // public void moveBackward() {
    // if (slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
    // slideMotor.setPower(-power);
    // } else {
    // stopMotor();
    // }
    // }
    //
    // public void goToPosition(int position) {
    // while (slideMotor.getCurrentPosition() < position) {
    // slideMotor.setPower(1);
    // }
    // stopMotor();
    // }
    //
    // public void goToRest() {
    // while (slideMotor.getCurrentPosition() > 5) {
    // slideMotor.setPower(-1);
    // }
    // stopMotor();
    // }

    // public void checkInputs(
    // Boolean extend,
    // Boolean retract,
    // Boolean reset
    // ) {
    // if (extend) {
    // moveForward();
    // } else if (retract) {
    // moveBackward();
    // } else {
    // stopMotor();
    // }
    // telemetry.addData("hSlide Power: ", slideMotor.getCurrent(CurrentUnit.AMPS));
    // telemetry.addData("hSlide Position: ", slideMotor.getCurrentPosition());
    //
    // if(reset) {
    // resetEncoder();
    // }
    //
    // }

    // public void resetEncoder() {
    //// while(slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
    //// moveBackward();
    //// telemetry.addData("moving back", "to reset encoder");
    //// telemetry.update();
    //// }
    //// telemetry.addData("hit limit", getPos());
    //// telemetry.update();
    //
    // slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    // slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    // }
    //
    // // Stops the slide
    // public void stopMotor() {
    // slideMotor.setPower(0);
    // }
    //
    //
    // public int getPos() {
    // return slideMotor.getCurrentPosition();
    // }
    //
    // public void setPower(double power) {
    // slideMotor.setPower(power);
    // }
}
