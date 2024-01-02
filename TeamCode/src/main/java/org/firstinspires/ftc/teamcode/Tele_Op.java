package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class Tele_Op extends LinearOpMode {


    //====================
    //create motors/servos

    //DT
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    //DT

    //LIFT
    public DcMotorEx ascending;
    DcMotor descending;
    //LIFT

    //BRAT
    Servo servoBrat;
    Servo stangaGripper;
    Servo dreaptaGripper;
    public DcMotorEx brat;

    //BRAT

    //INTAKE
    DcMotor intake;
    Servo dreaptaIntake;
    Servo stangaIntake;
    //INTAKE

    //PLANE
    Servo tureta;
    Servo avion;
    Servo unghiTureta;
    //PLANE

    public Arm Arm;
    public Lift lift;


    //create motors/servos
    //====================


    //====================
    //PID
    public static double viteza_lift = 80; // cm/s


    //PID
    //====================


    //====================
    //time
    public ElapsedTime elapsedTime;

    public ElapsedTime timerGripper;
    public ElapsedTime timerBrat;
    public ElapsedTime timerSBrat;
    public ElapsedTime timertureta;


    //time
    //====================


    //====================
    //state machine
    public enum StatusBrat {
        BRAT_SUS,
        BRAT_JOS
    }

    ;

    public enum Statusintake {
        ON,
        OFF
    }

    Old.StatusBrat PozitieBrat = Old.StatusBrat.BRAT_JOS;
    //state machine
    //====================


    @Override
    public void runOpMode() throws InterruptedException {


        //====================
        //PID


        //PID
        //====================


        //====================
        //motors/servos
        brat = hardwareMap.get(DcMotorEx.class, "brat");
        ascending = hardwareMap.get(DcMotorEx.class, "ascending");
        ascending = hardwareMap.get(DcMotorEx.class, "ascending");
        descending = hardwareMap.get(DcMotor.class, "descending");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        tureta = hardwareMap.get(Servo.class, "tureta");
        avion = hardwareMap.get(Servo.class, "avion");
        unghiTureta = hardwareMap.get(Servo.class, "unghiTureta");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");
        Arm = new Arm(brat);
        lift = new Lift(ascending);


        //motors/servos
        //====================


        //====================
        //configure motors/servos

        //set encoders


        //set target positions


        //set direction
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        ascending.setDirection(DcMotorSimple.Direction.REVERSE);
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

        //set zero power behavior
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set run mode

        //configure motors/servos
        //====================
        servoBrat.setPosition(0);
        stangaGripper.setPosition(0);
        dreaptaGripper.setPosition(0.8);
        tureta.setPosition(1);
        unghiTureta.setPosition(1);
        avion.setPosition(0.8);
        stangaIntake.setPosition(0);
        dreaptaIntake.setPosition(1);
        intake.setPower(0);

        //====================
        //runOpMode variables
        elapsedTime = new ElapsedTime();


        //runOpMode variables
        //====================


        waitForStart();
        elapsedTime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            //====================
            //while variables


            //while variables
            //====================


            //====================
            //telemetry


            //telemetry
            //====================

            //====================
            //PID


            //PID
            //====================


            //====================
            //set velocities


            //set velocities
            //====================


            //====================
            //functions
            if (gamepad2.dpad_up) {
                lift.setTargetPosition(lift.getTargetPosition() + elapsedTime.seconds() * viteza_lift);
            } else if (gamepad1.dpad_down) {
                lift.setTargetPosition(lift.getTargetPosition() - elapsedTime.seconds() * viteza_lift);
            }


            //functions
            //====================
            //intake
            if (gamepad1.right_trigger > .5) {

                stangaIntake.setPosition(0.44);
                dreaptaIntake.setPosition(0.56);
                intake.setPower(-1);
            }


            if (gamepad1.left_trigger > .5) {
                stangaIntake.setPosition(0.45);
                dreaptaIntake.setPosition(0.55);
                intake.setPower(1);
            }


            if (gamepad1.a) {
                stangaIntake.setPosition(0);
                dreaptaIntake.setPosition(1);

            }
            //intake

//BRAT

            switch (PozitieBrat) {

                case BRAT_JOS:

                    if (gamepad2.right_trigger > 0.5) {


                        if (stangaGripper.getPosition() != 0 || dreaptaGripper.getPosition() != .8) {
                            timerGripper.reset();
                        }
                        stangaGripper.setPosition(0);
                        dreaptaGripper.setPosition(.8);

                        if (timerGripper.seconds() > 0.3) {

                            brat.setTargetPosition(0);
                            servoBrat.setPosition(0);
                            if (servoBrat.getPosition() != 0) {
                                timerSBrat.reset();
                            }
                            servoBrat.setPosition(0);

                            if (timerGripper.seconds() > 0.2) {
                                brat.setTargetPosition(30);

                                if (servoBrat.getPosition() != 1) {
                                    timerSBrat.reset();
                                }
                                servoBrat.setPosition(1);

                                if (timerGripper.seconds() > 0.2) {
                                    brat.setTargetPosition(0);


                                    if (servoBrat.getPosition() != 0) {
                                        timerSBrat.reset();
                                    }
                                    servoBrat.setPosition(0);

                                    if (timerGripper.seconds() > 0.2) {

                                        if (stangaGripper.getPosition() != .12 || dreaptaGripper.getPosition() != .55) {
                                            timerGripper.reset();
                                        }
                                        stangaGripper.setPosition(.12);
                                        dreaptaGripper.setPosition(.55);

                                        if (timerGripper.seconds() > 0.2) {

                                            brat.setTargetPosition(350);
                                        }
                                    }
                                }
                            }
                        }



                        PozitieBrat = Old.StatusBrat.BRAT_SUS;
                    }

                    break;

                case BRAT_SUS:

                    if (gamepad2.left_trigger > 0.5) {
                        servoBrat.setPosition(1);
                        brat.setTargetPosition(0);
                        stangaGripper.setPosition(0);
                        dreaptaGripper.setPosition(.8);


                        PozitieBrat = Old.StatusBrat.BRAT_JOS;
                    }

                    break;


                default:


                    PozitieBrat = Old.StatusBrat.BRAT_JOS;
            }
            if (gamepad2.a) {
                stangaGripper.setPosition(0);
                dreaptaGripper.setPosition(0.8);

                if (brat.getTargetPosition() != 400) {
                    timerBrat.reset();
                }
                brat.setTargetPosition(400);

                if (timerBrat.seconds() > 0.1) {
                    brat.setTargetPosition(350);
                }
            }

            if (gamepad2.dpad_right) {
               stangaGripper.setPosition(0);
               dreaptaGripper.setPosition(0.8);
            }

            if (gamepad1.dpad_up) {
                tureta.setPosition(0);
                unghiTureta.setPosition(.5);
                if (unghiTureta.getPosition() != .5) {
                    timertureta.reset();
                }
                unghiTureta.setPosition(.5);

                if (timertureta.seconds() > 0.4) {
                    avion.setPosition(.1);
                }
            }






                //====================
                //debugging
                Arm.update();
                if (!gamepad1.y) {//daca NU se ridica
                    lift.update();
                    //ca sa dai overwrite la puterea motorului, trebuie sa efectuezi comanda DUPA lift.update();
                }
                //debugging
                //====================

                //DT
                //====================
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;


                double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn)));
                leftFront.setPower((forward + strafe + turn) / denominator * 2); //Math.min(1,(forward+ strafe + turn) / denominator *1.22))
                leftRear.setPower((forward - (strafe - turn)) / denominator * 2);
                rightFront.setPower((forward - (strafe + turn)) / denominator * 2);
                rightRear.setPower((forward + (strafe - turn)) / denominator * 2);

                //====================


                elapsedTime.reset();

            }//ends while
        }//ends runOpMode
    }//ends class

