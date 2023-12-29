package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OutTake;

public class Robot {

    public static void sleep(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    DcMotor intake;
    Servo dreaptaIntake;
    Servo stangaIntake;
    Servo servoBrat;
    DcMotor brat;
    Servo stangaGripper;
    Servo dreaptaGripper;
    Servo tureta;
    Servo avion;
    Servo unghiTureta;

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor ascending;
    DcMotor descending;
    public enum StatusBrat{
        BRAT_SUS,
        BRAT_JOS};
    public  enum Statusintake {
        ON,
        OFF
    }

    TeleOp.StatusBrat PozitieBrat = TeleOp.StatusBrat.BRAT_SUS;



    public Robot(HardwareMap hardwareMap) {
        ascending = hardwareMap.get(DcMotor.class, "ascending");
        descending = hardwareMap.get(DcMotor.class, "descending");
        ascending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascending.setTargetPosition(0);
        ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        descending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        descending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        descending.setTargetPosition(0);
        descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");



        ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");

        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");

        tureta = hardwareMap.get(Servo.class, "tureta");
        avion = hardwareMap.get(Servo.class, "avion");
        unghiTureta = hardwareMap.get(Servo.class, "unghiTureta");

        servoBrat = hardwareMap.get(Servo.class, "servoBrat");

        brat = hardwareMap.get(DcMotorEx.class,"brat");
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }


}
