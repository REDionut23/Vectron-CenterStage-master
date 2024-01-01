package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LiftTest2 extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        hardwareMap.get(DcMotor.class, "descending").setPower(1);
    }
}
