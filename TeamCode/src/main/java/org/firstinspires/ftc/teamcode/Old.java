package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;


import org.firstinspires.ftc.robotcore.external.JavaUtil;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "teleop")

public class Old extends OpMode {
    Robot robot;

   // private DcMotor brat;
    //private PIDController controllerb;

    //public static double pb = 0.07, ib = 0, db = 0.03;
   // public static double fb = 0.2;

   // public static int targetb = 10;
   // private final double ticks_in_degreeb = 1.493;

DcMotor brat;
    private DcMotor ascending;
    private DcMotor descending;
    private PIDController controller;

    public static double p = 0.06, i = 0, d = 0.00001;
    public static double f = 0.2;

    public static int target = 0;
    private final double ticks_in_degree = 4.666666666666667;




    public enum StatusBrat {
        BRAT_SUS,
        BRAT_JOS
    }

    ;

    public enum Statusintake {
    }

    Old.StatusBrat PozitieBrat = Old.StatusBrat.BRAT_SUS;

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
        brat = hardwareMap.get(DcMotor.class, "brat");

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       brat.setTargetPosition(0);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);



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

        //init
        //controllerb = new PIDController(pb, ib, db);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //brat = hardwareMap.get(DcMotor.class, "brat");
        //brat.setDirection(DcMotorSimple.Direction.REVERSE);
        //brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //init
       controller = new PIDController(p, i, d);
       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
      ascending = hardwareMap.get(DcMotor.class, "ascending");
        descending = hardwareMap.get(DcMotor.class, "descending");
        ascending.setDirection(DcMotorSimple.Direction.REVERSE);


    }




    @Override
    public void loop() {


        //loop
        //controllerb.setPID(pb, ib, db);
        //int armPosb = brat.getCurrentPosition();
       // double pidb = controllerb.calculate(armPosb, targetb);
       // double ffb = Math.cos(Math.toRadians(targetb / ticks_in_degreeb)) * fb;

      //  double powerb = pidb + ffb;
       // brat.setPower(powerb);
//
       // telemetry.addData("pos ", armPosb);
       // telemetry.addData("target ", targetb);
       // telemetry.update();


        //loop
        controller.setPID(p, i, d);
       // int armPos = ascending.getCurrentPosition();
       // double pid = controller.calculate(armPos, target);
       // double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

       // double power = pid + ff;
       // ascending.setPower(power);
       // descending.setPower(-power);


       // telemetry.addData("pos ", armPos);
       // telemetry.addData("target ", target);
        //telemetry.update();

//if(gamepad1.dpad_down)
{
  //brat.setTargetPosition(350);
    //brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);


}

        //if(gamepad1.dpad_up)
        {
           // brat.setTargetPosition(0);
           // brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }


        if(gamepad1.dpad_left)
        {


        }



        robot.intake.setPower(0);
        robot.servoBrat.setPosition(1);


        //DT

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;


        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
        robot.leftFront.setPower((forward + strafe + turn) / denominator * 2); //Math.min(1,(forward+ strafe + turn) / denominator *1.22))
        robot.leftRear.setPower((forward - (strafe - turn)) / denominator * 2);
        robot.rightFront.setPower((forward - (strafe + turn)) / denominator * 2);
        robot.rightRear.setPower((forward + (strafe - turn)) / denominator * 2);


        //INTAKE

        if (gamepad1.right_trigger > .5) {

            robot.stangaIntake.setPosition(0.44);
            robot.dreaptaIntake.setPosition(0.56);
            robot.intake.setPower(-1);
        }


        if (gamepad1.left_trigger > .5) {
            robot.stangaIntake.setPosition(0.45);
            robot.dreaptaIntake.setPosition(0.55);
            robot.intake.setPower(1);
        }


        if (gamepad1.a) {
            robot.stangaIntake.setPosition(0);
            robot.dreaptaIntake.setPosition(1);

        }


        //BRAT

        switch (robot.PozitieBrat) {

            case BRAT_JOS:

                if (gamepad2.right_trigger > 0.5) {
                    robot.stangaGripper.setPosition(0);
                    robot.dreaptaGripper.setPosition(0.8);


//coboara brat

                    robot.sleep(400);
                    robot.servoBrat.setPosition(0);

                    robot.sleep(400);
                    brat.setTargetPosition(-40);
                    brat.setPower(.7);

                    //prindere si ridicare
                    robot.servoBrat.setPosition(0.2);
                    brat.setTargetPosition(10);
                    brat.setPower(.7);


                    robot.sleep(300);
                    robot.dreaptaGripper.setPosition(0.55);
                    robot.stangaGripper.setPosition(0.12);
                    robot.servoBrat.setPosition(1);

                    robot.sleep(300);
                    brat.setTargetPosition(-350);
                    brat.setPower(.3);


                    robot.PozitieBrat = StatusBrat.BRAT_SUS;
                }

                break;

            case BRAT_SUS:

                if (gamepad2.left_trigger > 0.5) {
                    robot.stangaGripper.setPosition(0);
                    robot.dreaptaGripper.setPosition(0.8);
                    robot.servoBrat.setPosition(1);
                    brat.setTargetPosition(0);
                    brat.setPower(0.3);



                    robot.PozitieBrat = StatusBrat.BRAT_JOS;
                }

                break;


            default:


                robot.PozitieBrat = StatusBrat.BRAT_JOS;
        }

        if(gamepad2.dpad_left)
        {brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            brat.setTargetPosition(0);
            brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.a) {
            robot.stangaGripper.setPosition(0);
            robot.dreaptaGripper.setPosition(0.8);
            brat.setTargetPosition(-400);
            brat.setPower(1);
            brat.setTargetPosition(-350);
            brat.setPower(1);

        }

        if (gamepad2.dpad_right) {
            robot.stangaGripper.setPosition(0);
            robot.dreaptaGripper.setPosition(0.8);
        }

        //AVION




        if (gamepad2.x) {
            robot.tureta.setPosition(0);
            robot.unghiTureta.setPosition(.5);
        }


        //LIFT

        if (gamepad2.dpad_down) {
            robot.ascending.setTargetPosition(100);
            robot.ascending.setPower(1);





        }

        if (gamepad2.dpad_right) {
            robot.ascending.setTargetPosition(-4000);
            robot.ascending.setPower(1);





        }
        if (gamepad2.dpad_right) {
            robot.ascending.setTargetPosition(-6000);
            robot.ascending.setPower(1);





        }

        if (gamepad2.dpad_down) {
            robot.ascending.setPower(-1);
            robot.descending.setPower(.6);
            robot.sleep(2000);
            robot.ascending.setPower(0);
            robot.descending.setPower(0);


            robot.ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.descending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


    }


}