package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OutTake {
    public DcMotor brat;
    private final int downPos = 0;
    private final int upPos = 300;
    Servo dreaptaGripper;
    Servo stangaGripper;
    Servo servoBrat;

    private PIDController controller;

    public static double p = 0.1, i =0, d =0.00001;
    public static double f = 0.2;

    public static int target = 0;

    public double FlipStep = 0;

    private final double ticks_in_degree = 537.7 / 360;



    public OutTake(HardwareMap hardwareMap) {
        controller = new PIDController(p,i,d);
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");

    }
    public void loop() {


        controller.setPID(p, i, d);

        int armPos = brat.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = - Math.cos(Math.toRadians(target / ticks_in_degree)) * f;


        double power = pid + ff;

        FlipStep = brat.getCurrentPosition();//holds intake

        brat.setPower(power);


        // telemetry.addData("pos", armPos);
        // telemetry.addData("target", target);
        // telemetry.update();

    }//ends loop

    public void setTarget(int newTarget) {
        target = newTarget;
    }//ends setTarget



    public void bratup() {

        brat.setTargetPosition(upPos);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);

    }
    public void prepareCatch() {
        servoBrat.setPosition(0.5);
        brat.setTargetPosition(downPos);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brat.setPower(1);

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
