package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Lift {

    public static double p = 0.03, i = 0, d = 0, f = 0;

    public final static double cm_per_tick = 28 * 54.8;//28 * gear ratio final

    public final DcMotorEx lift_motor;
    public final PIDController controller = new PIDController(p, i, d);

    public static double target = 0;

    public Lift(DcMotorEx motor){
        lift_motor = motor;
    }

    public void setTargetPosition(double targetNou){
        target = targetNou;
    }

    public double getTargetPosition(){
        return target;
    }

    public void update(){
        controller.setPID(0.03,0,0);
        int armPos = lift_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = target * f;
        //la lift, gravitatia nu afecteaza pid-ul ca la brat(nu sinusoidal)
        //forta gravitationala este constanta

        double power = pid + ff;

        lift_motor.setPower(power);
    }
}