package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Arm {

    public static double p = 0.1, i = 0, d = 0.00001, f = 0.2;

    public final static double ticks_in_degree = 28 * 54.8;//28 * gear ratio final

    public final DcMotorEx brat;
    public final PIDController controller = new PIDController(p, i, d);

    public static double target = 0;

    public Arm(DcMotorEx motor){
        brat = motor;
    }

    public void setTargetPosition(double targetNou){
        target = targetNou;
    }

    public double getTargetPosition(){
        return target;
    }

    public void update(){
        controller.setPID(0.1,0,0.00001);
        int armPos = brat.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double power = pid + ff;

        brat.setPower(power);
    }
}