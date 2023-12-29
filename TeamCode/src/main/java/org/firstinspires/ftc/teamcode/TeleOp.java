package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;






import org.firstinspires.ftc.robotcore.external.JavaUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleop")

public class TeleOp extends OpMode {
    Robot robot;
    public enum StatusBrat{
        BRAT_SUS,
        BRAT_JOS};

    public  enum Statusintake {
    }

    TeleOp.StatusBrat PozitieBrat = TeleOp.StatusBrat.BRAT_SUS;

    ElapsedTime time;

    MultipleTelemetry tele;








    @Override
    public void init() {

        time = new ElapsedTime();
        tele = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        robot = new Robot(hardwareMap);


        robot.ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        robot.ascending = hardwareMap.get(DcMotor.class, "ascending");
        robot.descending = hardwareMap.get(DcMotor.class, "descending");
        robot.leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        robot.leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        robot.rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        robot.rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        robot.intake = hardwareMap.get(DcMotor.class, "intake");
        robot.stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        robot.dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        robot.stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        robot.dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        robot.tureta = hardwareMap.get(Servo.class, "tureta");
        robot.avion = hardwareMap.get(Servo.class, "avion");
        robot.unghiTureta = hardwareMap.get(Servo.class, "unghiTureta");
        robot.servoBrat = hardwareMap.get(Servo.class, "servoBrat");
        robot.brat = hardwareMap.get(DcMotorEx.class,"brat");


        robot.brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brat.setTargetPosition(0);
        robot.brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.ascending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ascending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ascending.setTargetPosition(0);
        robot.ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.ascending.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.descending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.descending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.descending.setTargetPosition(0);
        robot.descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.descending.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.servoBrat.setPosition(0);
        robot.stangaGripper.setPosition(0);
        robot.dreaptaGripper.setPosition(0.8);
        robot.tureta.setPosition(1);
        robot.unghiTureta.setPosition(1);
        robot.avion.setPosition(0.8);
        robot.stangaIntake.setPosition(0);
        robot.dreaptaIntake.setPosition(1);
        robot.intake.setPower(0);

    }
    @Override
    public void loop() {

        robot.intake.setPower(0);
        robot.servoBrat.setPosition(1);




        //DT

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
        robot.leftFront.setPower((forward + strafe + turn) / denominator*2); //Math.min(1,(forward+ strafe + turn) / denominator *1.22))
        robot.leftRear.setPower((forward - (strafe - turn)) / denominator*2);
        robot.rightFront.setPower((forward - (strafe + turn)) / denominator*2);
        robot.rightRear.setPower((forward + (strafe - turn)) / denominator*2);





        //INTAKE

        if (gamepad1.right_trigger > .5)
        {

            robot.stangaIntake.setPosition(0.44);
            robot.dreaptaIntake.setPosition(0.56);
            robot.intake.setPower(-1);
        }




        if (gamepad1.left_trigger > .5)
        {
            robot.stangaIntake.setPosition(0.45);
            robot.dreaptaIntake.setPosition(0.55);
            robot.intake.setPower(1);
        }



        if (gamepad1.a)
        {
            robot.stangaIntake.setPosition(0);
            robot.dreaptaIntake.setPosition(1);

        }




        //BRAT

        switch (robot.PozitieBrat) {

            case BRAT_SUS:

                if (gamepad2.left_bumper)
                {
                    robot.stangaGripper.setPosition(0);
                    robot.dreaptaGripper.setPosition(0.8);
                    robot.servoBrat.setPosition(0);
                    robot.sleep(400);
                    //ridica brat
                    robot.brat.setTargetPosition(-50);
                    robot.brat.setPower(.5);
                    robot.servoBrat.setPosition(1);
                    robot.sleep(400);
//coboara brat
                    robot.brat.setTargetPosition(30);
                    robot.brat.setPower(.5);
                    robot.sleep(500);
                    robot.servoBrat.setPosition(0);

                    robot.sleep(400);

                    //ridica brat
                    robot.brat.setTargetPosition(-50);
                    robot.brat.setPower(.5);
                    robot.servoBrat.setPosition(1);
                    robot.sleep(400);
//coboara brat
                    robot.brat.setTargetPosition( 30);
                    robot.brat.setPower(.5);
                    robot.sleep(500);
                    robot.servoBrat.setPosition(.2);

                    robot.sleep(400);


                    //prindere si ridicare
                    robot.servoBrat.setPosition(0);

                    robot.sleep(300);
                    robot.dreaptaGripper.setPosition(0.55);
                    robot.stangaGripper.setPosition(0.12);
                    robot.servoBrat.setPosition(1);
                    robot.sleep(600);
                    robot.brat.setTargetPosition(-350);
                    robot.brat.setPower(.5);

                    robot.PozitieBrat = StatusBrat.BRAT_SUS;
                }

                break;

            case BRAT_JOS:

                if (gamepad2.right_bumper)
                {
                    robot.stangaGripper.setPosition(0);
                    robot.dreaptaGripper.setPosition(0.8);
                    robot.servoBrat.setPosition(1); // aici tu stii cum ai pozitiile eu pun 0 si 1
                    robot.brat.setTargetPosition(30);
                    robot.brat.setPower(.5);

                    robot.PozitieBrat = StatusBrat.BRAT_JOS;
                }

                break;


            default:


                robot.PozitieBrat = StatusBrat.BRAT_SUS;
        }



        if (gamepad2.dpad_right)
        {
            robot.stangaGripper.setPosition(0);
            robot.dreaptaGripper.setPosition(0.8);
        }

        //AVION

        if (gamepad2.x)
        {
            robot.tureta.setPosition(0);
            robot.unghiTureta.setPosition(.5);
        }


        //LIFT

        if (gamepad2.dpad_up)
        {
            robot.ascending.setPower(1);
            robot.descending.setPower(-0.6);
            robot.sleep(2000);
            robot.ascending.setPower(0);
            robot.descending.setPower(0);

            robot.ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (gamepad2.dpad_down)
        {
            robot.ascending.setPower(-1);
            robot.descending.setPower(.6);
            robot.sleep(2000);
            robot.ascending.setPower(0);
            robot.descending.setPower(0);


            robot.ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}




    }


}
