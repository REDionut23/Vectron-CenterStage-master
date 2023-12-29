package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Lift {
    public DcMotorEx ascending;
    public DcMotorEx descending;

    public Lift(@NonNull HardwareMap hardwareMap) {
        ascending = hardwareMap.get(DcMotorEx.class, "ascending");
        descending = hardwareMap.get(DcMotorEx.class, "descending");

//        ascending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        descending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        ascending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        descending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void ascend() {
        ascending.setPower(-1);
        descending.setTargetPosition(ascending.getCurrentPosition());
        descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        descending.setPower(1);
    }

    public void descend() {
        ascending.setPower(1);
        descending.setTargetPosition(ascending.getCurrentPosition());
        descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        descending.setPower(1);
    }

    public void stop() {
        ascending.setPower(0);
        descending.setPower(0);
    }

}
