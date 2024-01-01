package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CS_Blue1_Pipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.OutTake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Autonomous(name = "autoBL")  // ----------------------------------------------------------------- Program name here
//          And here
public class autoBL extends LinearOpMode
{

    //====================
    //armPID



    //armPID
    //====================


OutTake outTake;
    public static void sleep(int ms)
    {
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }


    DcMotor intake;
    Servo dreaptaIntake;
    Servo stangaIntake;
    Servo servoBrat;

    Servo stangaGripper;
    Servo dreaptaGripper;
    Servo tureta;
    Servo avion;
    Servo unghiTureta;

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor ascending;
    DcMotor descending;







    OpenCvWebcam webcam;


    /************************************************************************
     * CALIBRATION CONSTANTS:
     *   Use for common servo positions, motor tick scaling, motor limits,
     *   default times, default powers.
     *   This allows the code to be adjusted only in this section, if
     *   a servo is mounted in a different orientation, for example. -------------------------------------- Adjust constants where necessary
     ************************************************************************/
 /*
  double GRABBER_SERVO_OPENED_POS = 0.37;//0.45;
  double GRABBER_SERVO_CLOSED_POS = 0.163;//0.12;
  double GRABBER_SERVO_OPEN_A_LITTLE_POS = 0.206;//0.22;
  int GRABBER_SERVO_PAUSE_TIME_MS = 355;//450;

  double ARM_RAISE_TICKS_PER_DEG = 7.595;
  double ARM_EXTEND_TICKS_PER_INCH = 120.0;

  double CLAW_FLIP_SERVO_NORMAL_POS = 0.263;//0.27;
  double CLAW_FLIP_SERVO_TO_FROM_GROUND = 0.24;
  double CLAW_FLIP_SERVO_FLIPPED_POS = 0.93;
  int FLIPPER_SERVO_PAUSE_TIME_MS = 910;//1000;
  int CALC_GRABBER_WAIT_MS = 1000;
  int CALC_FLIPPER_WAIT_MS = 1500;

  double DRIVE_POWER = 0.5;
*/

    /**
     * This function is executed when this OpMode is selected & Init is pressed
     */
    @Override
    public void runOpMode() throws InterruptedException

    {



//====================




        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
       dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");







        //brat = hardwareMap.get(DcMotorEx.class,"brat");
        // brat.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */

        ascending = hardwareMap.get(DcMotorEx.class,"ascending");
        ascending.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ascending.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ascending.setTargetPosition(0);
        ascending.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ascending.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ;        /************************************************************************
         * INIT CODE:
         *   -map hardware specific to our robot
         *   -config hardware settings (like reversed motor directions)
         *   -config camera & OpenCV pipeline used for prop location detection  ---------------------------------- Set motor variables
         ************************************************************************/



        // detectare cu webcam
        int camMonViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), camMonViewId);

        // specify image processing pipeline to invoke


        // upon receipt of a frame from the camera
        CS_Blue1_Pipeline findPropPL = new CS_Blue1_Pipeline(telemetry);
        webcam.setPipeline(findPropPL);

        // open connection to webcam asynchronously
        webcam.setMillisecondsPermissionTimeout(5000);

        // optional: use GPU acceleration
        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(800, 600, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        while (!opModeIsActive() & !isStopRequested())
        {
            if (findPropPL.enableDetection)
            {
                telemetry.addLine("Waiting for start");
                telemetry.addData("PROP LOCATION: ", findPropPL.getLocation());
                telemetry.update();
            }
        }

        findPropPL.enableDetection = false;

        // wait for user to press start on Driver Station
        Pose2d startPose = new Pose2d(11.5, 60.15, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(15),

                new AngularVelocityConstraint(3)


        ));
        waitForStart();







        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> dreaptaGripper.setPosition(0.55))
                .addTemporalMarker(() -> stangaGripper.setPosition(0.12))


                .addTemporalMarker(() -> dreaptaIntake.setPosition(0.56)) // Lower servo
                .addTemporalMarker(() -> stangaIntake.setPosition(0.44)) // Lower servo
                .waitSeconds(1)
                .lineTo(new Vector2d(11.50, 50))



                .build();
        // --------------------------------------------------------------------------------------------------- your tele-op code here
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            switch (CS_Blue1_Pipeline.location){
                case LEFT:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .splineToConstantHeading(new Vector2d(25.12, 34.21), Math.toRadians(-90.00))
                            .addTemporalMarker(() -> dreaptaIntake.setPosition(1)) // Lower servo
                            .addTemporalMarker(() -> stangaIntake.setPosition(0)) // Lower servo
                            .waitSeconds(2)



                            .lineTo(new Vector2d(25.12, 50.03))

                            .addTemporalMarker(() -> servoBrat.setPosition(1))



                            .addTemporalMarker(() -> ascending.setTargetPosition(1000))
                            .addTemporalMarker(() -> ascending.setPower(1))

                            .waitSeconds(2)
                            .splineToLinearHeading(new Pose2d(53.54, 44.17, Math.toRadians(185.00)), Math.toRadians(0.00))
                            .addTemporalMarker(() -> outTake.bratup())
                            .waitSeconds(2)

                            .addTemporalMarker(() -> dreaptaGripper.setPosition(0.8))
                            .addTemporalMarker(() -> stangaGripper.setPosition(0))

                            .waitSeconds(2)

                            .splineToConstantHeading(new Vector2d(41.97, 59.11), Math.toRadians(0.00))
                            .addTemporalMarker(() -> ascending.setTargetPosition(-10))
                            .addTemporalMarker(() -> ascending.setPower(1))

                            .waitSeconds(2)
                            .lineTo(new Vector2d(59.99, 61.75))







                            .build());
                    break;
                case CENTER:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .splineToConstantHeading(new Vector2d(11.50, 27.61), Math.toRadians(270.00))
                            .addTemporalMarker(() -> dreaptaIntake.setPosition(1)) // Lower servo
                            .addTemporalMarker(() -> stangaIntake.setPosition(0)) // Lower servo
                            .waitSeconds(2)



                            .addTemporalMarker(() -> servoBrat.setPosition(1))
                            .addTemporalMarker(() -> outTake.bratup())
                            .addTemporalMarker(() -> ascending.setTargetPosition(1000))
                            .addTemporalMarker(() -> ascending.setPower(1))
                            .waitSeconds(2)

                            .lineToSplineHeading(new Pose2d(54.57, 35.82, Math.toRadians(180.00)))


                            .addTemporalMarker(() -> dreaptaGripper.setPosition(0.8))
                            .addTemporalMarker(() -> stangaGripper.setPosition(0))

                            .waitSeconds(2)



                            .splineToConstantHeading(new Vector2d(36.99, 59.26), Math.toRadians(0.00))


                            .addTemporalMarker(() -> ascending.setTargetPosition(-10))
                            .addTemporalMarker(() -> ascending.setPower(1))

                            .waitSeconds(2)

                            .lineTo(new Vector2d(60.13, 59.40))


















                            .build());

                    break;
                case RIGHT:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .splineToSplineHeading(new Pose2d(6, 30.54, Math.toRadians(180.00)), Math.toRadians(246.50))
                            .addTemporalMarker(() -> dreaptaIntake.setPosition(1)) // Lower servo
                            .addTemporalMarker(() -> stangaIntake.setPosition(0)) // Lower servo
                            .waitSeconds(2)
                            .addTemporalMarker(() -> servoBrat.setPosition(1))
                            .addTemporalMarker(() -> outTake.bratup())
                            .addTemporalMarker(() -> ascending.setTargetPosition(1000))
                            .addTemporalMarker(() -> ascending.setPower(1))
                            .waitSeconds(2)

                            .lineToConstantHeading(new Vector2d(55.74, 29.22))

                            .addTemporalMarker(() -> dreaptaGripper.setPosition(0.8))
                            .addTemporalMarker(() -> stangaGripper.setPosition(0))

                            .waitSeconds(2)

                            .lineToConstantHeading(new Vector2d(39.19, 58.67))
                            .addTemporalMarker(() -> ascending.setTargetPosition(-10))
                            .addTemporalMarker(() -> ascending.setPower(1))

                            .waitSeconds(2)

                            .lineTo(new Vector2d(62.33, 58.67))







                            .build());



                    break;


            }
// Put loop blocks here.

                // do some telemetry
                telemetry.addData("PROP LOCATION: ", findPropPL.location);

                telemetry.update();

                // don't do this in a real op mode, but for this cam test, chill on the CPU cycles
            }




            // Put loop blocks here.

            // do some telemetry
            telemetry.addData("PROP LOCATION: ", findPropPL.location);

            telemetry.update();

            // don't do this in a real op mode, but for this cam test, chill on the CPU cycles

        }
    }

