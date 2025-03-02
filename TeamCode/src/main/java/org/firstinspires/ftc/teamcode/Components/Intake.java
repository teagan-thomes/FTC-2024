package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Servo leftWrist;
    private Servo rightWrist;
    private Servo leftGrabber;
    private Servo rightGrabber;

    private HorizontalSlide hSlide;

    public Servo RGB;
    public HuskyLens huskyLens;

    private OpMode opMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;


    int maxFlipPosition = 525;
    int minFlipPosition = 0;

    static String wristPosition = "up";



    public Intake(OpMode opMode, HorizontalSlide hSlide, MainDrive mainDrive) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        this.hSlide = hSlide;
        leftWrist = opMode.hardwareMap.get(Servo.class, "leftWrist");
        rightWrist = opMode.hardwareMap.get(Servo.class, "rightWrist");
        leftGrabber = opMode.hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = opMode.hardwareMap.get(Servo.class, "rightGrabber");

        leftWrist.setPosition(0);
        rightWrist.setPosition(1);
        leftGrabber.setPosition(0.5);
        rightGrabber.setPosition(0.5);

        RGB = this.hardwareMap.get(Servo.class, "light");
        RGB.setPosition(0);

        huskyLens = this.hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void checkInputs(
            Boolean wristDown,
            Boolean wristUp,
            Boolean wristHalf,
//            Boolean wristModifier,
            Boolean grabberSuck,
            Boolean grabberSpit
    ) {
        double wristIncrement = .015;

        if(hSlide.getPos() > 0.3) {
            if (wristDown && (hSlide.getPos() < .85) && !gamepad1.start) {
                wristDown();
                wristPosition = "down";
            }

            else if (wristUp && !gamepad1.start) {
                wristUp();
                wristPosition = "up";
            }

            else if (wristHalf) {
                wristHalf();
                wristPosition = "half";
            }
        }
        //TODO CHANGE THIS FOR AXON SERVOS
//        if((hSlide.getPos() < maxFlipPosition)) {
//            if (wristDown && (hSlide.getPos() > minFlipPosition) && !gamepad1.start) { // && !wristModifier
//                wristDown();
//            }
//            else if (wristUp && !gamepad1.start) { // && !wristModifier
//                wristUp();
//            }
//            else if (wristHalf) {
//                wristHalf();
//            }

//            if(wristModifier && wristUp) {
//                setWristPosition(leftWrist.getPosition() - wristIncrement, rightWrist.getPosition() + wristIncrement);
//
//            }
//            else if(wristModifier && wristDown) {
//                setWristPosition(leftWrist.getPosition() + wristIncrement, rightWrist.getPosition() - wristIncrement);
//            }
//
//            telemetry.addData("left wrist pos", leftWrist.getPosition());
//            telemetry.addData("right wrist pos",  rightWrist.getPosition());
//        }
//        else {
//            telemetry.addData("Wrist", "Cannot move wrist down, horizontal slide is too high");
//        }

        if (grabberSuck) {
            grabberSuck();
        } else if (grabberSpit) {
            grabberSpit();
        } else {
            grabberOff();
        }

        updateLight();

    }

    // SERVO CONTROLLER INFO: leftWrist:        left limit = 1, right limit = 0     rightWrist: left limit = 0, right limit = 1
    public void setWristPosition(double leftPosition, double rightPosition) {
        leftWrist.setPosition(leftPosition);
        rightWrist.setPosition(rightPosition);
    }

    public void setGrabberPosition(double leftPosition, double rightPosition) {
        leftGrabber.setPosition(leftPosition);
        rightGrabber.setPosition(rightPosition);
    }

    public static String getWristPosition() {
        return wristPosition;
    }

    public void wristDown() {
        setWristPosition(.91, .32); // .91 and .32
    }

    public void wristUp() {
        setWristPosition(0, 1);
    }

    public void wristHalf() {
        setWristPosition(0.6, 0.65);
    } // .6 and .4

    public void grabberSuck() {
        setGrabberPosition(0, 1);
    }

    public void grabberSpit() {
        setGrabberPosition(1, 0);
    }

    public void grabberOff() {
        setGrabberPosition(0.5, 0.5);
    }

    // Add a confidence counter
    int confidenceCounter = 0;
    int lastDetectedId = -1;

    void updateLight() {
        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks.length > 0) {
            HuskyLens.Block largestBlock = blocks[0];

            for (HuskyLens.Block block : blocks) {
                int currentArea = block.width * block.height;
                int largestArea = largestBlock.width * largestBlock.height;

                if (currentArea > largestArea) {
                    largestBlock = block;
                }
            }

            int detectedId = largestBlock.id;



            // Ignore ID 4
                // Increase confidence counter if the same ID is detected multiple times
            if (detectedId == lastDetectedId) {
                confidenceCounter++;
            } else {
                confidenceCounter = 0;  // Reset if ID changes
            }

            lastDetectedId = detectedId;

            if (confidenceCounter > 10) {
                if (detectedId == 1) {
                    RGB.setPosition(0.279);
                } else if (detectedId == 2) {
                    RGB.setPosition(0.388);
                } else if (detectedId == 3) {
                    RGB.setPosition(0.611);
                } else if (detectedId == 4) {
                    RGB.setPosition(0);
                } else {
                    RGB.setPosition(0);
                }
            }
        } else {
            RGB.setPosition(0);
        }

        telemetry.update();
    }


}