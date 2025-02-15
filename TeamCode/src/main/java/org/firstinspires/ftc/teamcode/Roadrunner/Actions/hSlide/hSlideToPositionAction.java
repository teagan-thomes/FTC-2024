package org.firstinspires.ftc.teamcode.Roadrunner.Actions.hSlide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.Components.HorizontalSlide;

public class hSlideToPositionAction implements Action {
    private HorizontalSlide hSlide;
    private double leftPos;
    private double rightPos;

    public hSlideToPositionAction(HorizontalSlide hSlide, double leftPos, double rightPos) {
        this.hSlide = hSlide;
        this.leftPos = leftPos;
        this.rightPos = rightPos;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        hSlide.actionControl(leftPos, rightPos);
        return false;
    }
}
