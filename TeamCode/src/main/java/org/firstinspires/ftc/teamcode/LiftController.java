package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import  com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class LiftController extends OpMode{
    private PIDController controllerb;

    public static double pb = 0.1, ib =0, db =0.00001;
    public static double fb = 0.2;

    public static int targetb =0;
    private final double ticks_in_degree = 1.493;

    public DcMotor brat;
    @Override
    public void init() {
        controllerb = new PIDController(pb, ib, db);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance(). getTelemetry());
        brat = hardwareMap.get(DcMotor.class, "brat");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controllerb.setPID(pb, ib, db);
        int armPos =brat.getCurrentPosition();
        double pid = controllerb.calculate(armPos, targetb);
        double ff = Math.cos(Math.toRadians(targetb/ticks_in_degree)) *fb;

        double power = pid +ff;
        brat.setPower(power);
        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", targetb );
        telemetry.update();

    }
}