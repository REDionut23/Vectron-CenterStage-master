package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {

    public Servo tureta;
    public Servo unghiTureta;
    public Servo avion;

    public Launcher(HardwareMap hardwareMap) {
        tureta = hardwareMap.get(Servo.class, "tureta");
        unghiTureta = hardwareMap.get(Servo.class, "unghiTureta");
        avion = hardwareMap.get(Servo.class, "avion");
        avion.setPosition(1);
    }

    public void prepareLaunch() {
        tureta.setPosition(0.5);
        unghiTureta.setPosition(0.8);
    }

    public void airplaneIn() {
        tureta.setPosition(1);
        unghiTureta.setPosition(1);
        avion.setPosition(1);
    }

    public void launch() {
        avion.setPosition(0.5);
    }
}
