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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Objects;

public class HorizontalSlide {
//    private DcMotorEx slideMotor;
    private Servo leftLinkage;
    private Servo rightLinkage;

    private final double power;
    private final double currentLimit;

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    double speed = 0.01;


//    int maxSlidePosition = 650; // 675
//    private final Gamepad gamepad1;
//    private final Gamepad gamepad2;


    public HorizontalSlide(OpMode opMode, double currentLimit) {
//        slideMotor = opMode.hardwareMap.get(DcMotorEx.class, "slideMotor");
//        slideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftLinkage = opMode.hardwareMap.get(Servo.class, "leftLinkage");
        rightLinkage = opMode.hardwareMap.get(Servo.class, "rightLinkage");

        this.power = 1;
        this.currentLimit = currentLimit;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;

    }
    public void moveForward() {
        leftLinkage.setPosition(0.34);
        rightLinkage.setPosition(0.3);
    }
    public void moveBackward() {
        leftLinkage.setPosition(0.9);
        rightLinkage.setPosition(0.94);
    }
    public void moveForwardGradually() {
//        leftLinkage.setPosition(Math.max(0.14, leftLinkage.getPosition() - speed));
//        rightLinkage.setPosition(Math.max(0.1, rightLinkage.getPosition() - speed));
        leftLinkage.setPosition(leftLinkage.getPosition() - speed);
        rightLinkage.setPosition(rightLinkage.getPosition() - speed);
    }
    public void moveBackwardGradually() {
//        leftLinkage.setPosition(Math.min(0.9, leftLinkage.getPosition() + speed));
//        rightLinkage.setPosition(Math.min(0.94, rightLinkage.getPosition() + speed));
        leftLinkage.setPosition(leftLinkage.getPosition() + speed);
        rightLinkage.setPosition(rightLinkage.getPosition() + speed);;
    }

    public void checkInputs(boolean extend, boolean retract, double extendGradual, double retractGradual) {
        if (extend) {
            moveForward();
        }
        else if (retract && Objects.equals(Intake.getWristPosition(), "up")) {
            moveBackward();
        }
        else {
            // GRADUAL movement using triggers
            if (extendGradual > 0.1) {
                moveForwardGradually();
            }
            if (retractGradual > 0.1) {
                moveBackwardGradually();
            }
        }
    }

    public double getPos() {
        return leftLinkage.getPosition();
    }

//    public void moveForward() {
//        if (slideMotor.getCurrentPosition() < maxSlidePosition) {
//            slideMotor.setPower(power);
//        } else {
//            stopMotor();
//        }
//    }
//
//    public void moveBackward() {
//        if (slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
//            slideMotor.setPower(-power);
//        } else {
//            stopMotor();
//        }
//    }
//
//    public void goToPosition(int position) {
//        while (slideMotor.getCurrentPosition() < position) {
//            slideMotor.setPower(1);
//        }
//        stopMotor();
//    }
//
//    public void goToRest() {
//        while (slideMotor.getCurrentPosition() > 5) {
//            slideMotor.setPower(-1);
//        }
//        stopMotor();
//    }

//    public void checkInputs(
//            Boolean extend,
//            Boolean retract,
//            Boolean reset
//    ) {
//        if (extend) {
//            moveForward();
//        } else if (retract) {
//            moveBackward();
//        } else {
//            stopMotor();
//        }
//        telemetry.addData("hSlide Power: ", slideMotor.getCurrent(CurrentUnit.AMPS));
//        telemetry.addData("hSlide Position: ", slideMotor.getCurrentPosition());
//
//        if(reset) {
//            resetEncoder();
//        }
//
//    }

//    public void resetEncoder() {
////        while(slideMotor.getCurrent(CurrentUnit.AMPS) < currentLimit) {
////            moveBackward();
////            telemetry.addData("moving back", "to reset encoder");
////            telemetry.update();
////        }
////        telemetry.addData("hit limit", getPos());
////        telemetry.update();
//
//        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//    }
//
//    // Stops the slide
//    public void stopMotor() {
//        slideMotor.setPower(0);
//    }
//
//
//    public int getPos() {
//        return slideMotor.getCurrentPosition();
//    }
//
//    public void setPower(double power) {
//        slideMotor.setPower(power);
//    }
}
