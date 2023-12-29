package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutTake {
    public DcMotor brat;
    private final int downPos = 0;
    private final int upPos = -280;
    Servo dreaptaGripper;
    Servo stangaGripper;
    Servo servoBrat;

    public OutTake(HardwareMap hardwareMap) {
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void prepareCatch() {
        servoBrat.setPosition(0.5);
        brat.setTargetPosition(downPos);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
        releasePixels();
    }

    public void prepareRelease() {
        servoBrat.setPosition(0.5);
        brat.setTargetPosition(upPos);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);
    }

    public void catchPixels() {
//        dreaptaGripper.setPosition(0.55);
//        stangaGripper.setPosition(0.1);
    }

    public void releasePixels() {
//        stangaGripper.setPosition(0);
//        dreaptaGripper.setPosition(0.8);
    }
}
