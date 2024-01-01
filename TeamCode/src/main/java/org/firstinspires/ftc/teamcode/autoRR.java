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
import org.firstinspires.ftc.teamcode.CS_1_Pipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;


@Autonomous(name = "autoRR")  // ----------------------------------------------------------------- Program name here
//          And here
public class autoRR extends LinearOpMode
{



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
    DcMotor  brat;







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
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        stangaIntake = hardwareMap.get(Servo.class, "stangaIntake");
        dreaptaIntake = hardwareMap.get(Servo.class, "dreaptaIntake");
        stangaGripper = hardwareMap.get(Servo.class, "stangaGripper");
        dreaptaGripper = hardwareMap.get(Servo.class, "dreaptaGripper");
        servoBrat = hardwareMap.get(Servo.class, "servoBrat");



        brat = hardwareMap.get(DcMotorEx.class,"brat");
        brat.setDirection(DcMotorSimple.Direction.REVERSE);

        brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setTargetPosition(0);
        brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        CS_1_Pipeline findPropPL = new CS_1_Pipeline(telemetry);
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
        Pose2d startPose = new Pose2d(11.79, -62.33, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(

                new TranslationalVelocityConstraint(13),

                new AngularVelocityConstraint(3)


        ));
        waitForStart();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> dreaptaGripper.setPosition(0.55))
                .addTemporalMarker(() -> stangaGripper.setPosition(0.12))


                .addTemporalMarker(() -> dreaptaIntake.setPosition(0.56)) // Lower servo
                .addTemporalMarker(() -> stangaIntake.setPosition(0.44)) // Lower servo
                .waitSeconds(1)
                .lineTo(new Vector2d(11.79, -59.83))



                .build();
        // --------------------------------------------------------------------------------------------------- your tele-op code here
        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            switch (CS_1_Pipeline.location){
                case LEFT:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .splineToSplineHeading(new Pose2d(3.74, -30.69, Math.toRadians(180.00)), Math.toRadians(90.25))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(55.74, -30.98))

                            .waitSeconds(2)
                            .splineToConstantHeading(new Vector2d(37.57, -58.96), Math.toRadians(0.00))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(62.92, -58.82))






                            .build());
                    break;
                case CENTER:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .lineTo(new Vector2d(11.79, -28.93))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(11.65, -41.68))
                            .waitSeconds(2)
                            .splineToLinearHeading(new Pose2d(57.06, -36.11, Math.toRadians(180.00)), Math.toRadians(0.00))
                            .waitSeconds(2)
                            .splineToConstantHeading(new Vector2d(35.67, -59.69), Math.toRadians(0.00))
                            .waitSeconds(2)
                            .lineTo(new Vector2d(58.08, -61.60))

                            .waitSeconds(2)

















                            .build());

                    break;
                case RIGHT:
                    drive.followTrajectorySequence(drive.trajectorySequenceBuilder(trajSeq.end())
                            .splineToSplineHeading(new Pose2d(17.07, -31.13, Math.toRadians(0.00)), Math.toRadians(0.00))

                            .waitSeconds(2)

                            .splineToConstantHeading(new Vector2d(9.30, -35.23), Math.toRadians(241.67))
                            .waitSeconds(2)

                            .splineToSplineHeading(new Pose2d(24.10, -47.98, Math.toRadians(180.00)), Math.toRadians(0.00))
                            .waitSeconds(2)

                            .splineToSplineHeading(new Pose2d(53.84, -42.56, Math.toRadians(180.00)), Math.toRadians(0.00))
                            .waitSeconds(2)

                            .splineToConstantHeading(new Vector2d(36.84, -58.82), Math.toRadians(0.00))
                            .waitSeconds(2)

                            .lineTo(new Vector2d(62.19, -60.72))
                            .waitSeconds(2)









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

