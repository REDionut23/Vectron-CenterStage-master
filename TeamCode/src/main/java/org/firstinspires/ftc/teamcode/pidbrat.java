package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class pidbrat{

    Robot robot;
    static DcMotorEx brat;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0.0001;
    public static int bratTargetPos = 0;
    private final double ticks_in_degree = 192.25/180.0;
    public static PIDController pid;
    // Creates a PIDFController with gains kP, kI, kD, and kF
    public void init(DcMotorEx slide1, DcMotorEx slide2) {
        brat= brat;

        pid = new PIDController(kP, kI, kD);
        //leftSlide = (DcMotorEx) hardwareMap.dcMotor.get("LeftSlide");
        //rightSlide = (DcMotorEx) hardwareMap.dcMotor.get("RightSlide");

        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      brat.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftTargetPos = 0;
    }
    public void loop(){
        pid.setPID(kP,kI,kD);
        double pidPower = pid.calculate(brat.getCurrentPosition(), bratTargetPos);
        double ffPower = kF * brat.getCurrentPosition();
        brat.setPower(ffPower + pidPower);



    }
    public void changeHeight(int pos){
        bratTargetPos = pos;
    }
    public void reset(){
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        brat .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        bratTargetPos=0;
    }
}