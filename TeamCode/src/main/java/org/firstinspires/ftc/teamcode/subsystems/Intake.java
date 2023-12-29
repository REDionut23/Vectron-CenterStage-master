package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotor intake;
    public Servo stangaIntake;
    public Servo dreaptaIntake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        stangaIntake.setPosition(0.48);
    }

    public void resIntake() {
        dreaptaIntake.setPosition(0.13);
        stangaIntake.setPosition(0.87);

    }

    public void lowIntake() {
        dreaptaIntake.setPosition(0.3);
        stangaIntake.setPosition(0.1);

    }

    public void takeIn() {
        intake.setPower(1);
    }

    public void takeOut() {
        intake.setPower(-1);
    }

    public void stop() {
        intake.setPower(0);
    }
}
