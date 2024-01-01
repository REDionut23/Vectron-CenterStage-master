package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class pidbrat extends OpMode {
    private PIDController controller;

    public static double p = 0.1, i =0, d =0.00001;
    public static double f = 0.2;

    public static int target =0;
    private final double ticks_in_degree = 1.493;

    public DcMotor brat;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance(). getTelemetry());
        brat = hardwareMap.get(DcMotor.class, "brat");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
controller.setPID(p, i, d);
int armPos =brat.getCurrentPosition();
double pid = controller.calculate(armPos, target);
double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) *f;


double power = pid +ff;
brat.setTargetPosition(50);
brat.setPower(power);
telemetry.addData("pos ", armPos);
telemetry.addData("target ", target );
telemetry.update();

    }
}