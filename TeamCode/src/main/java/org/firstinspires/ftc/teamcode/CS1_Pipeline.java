package org.firstinspires.ftc.teamcode;

import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;


import java.lang.Math;


public class CS1_Pipeline extends OpenCvPipeline {
    public enum Location {
        LEFT,
        CENTER,
        RIGHT
    }
    private Location location;

    Telemetry telemetry;

    static final Rect LEFT_ROI = new Rect(
            new Point(15, 150),
            new Point(35, 170));

    static final Rect CENTER_ROI = new Rect(
            new Point(110, 140),
            new Point(130, 160));

    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public CS1_Pipeline(Telemetry telemetry)
    { this.telemetry = telemetry; }


    @Override
    public Mat processFrame(Mat input) {
        //Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2GRAY);

        Mat left = input.submat(LEFT_ROI);
        Mat center = input.submat(CENTER_ROI);

        double leftValue = Core.sumElems(left).val[1] / LEFT_ROI.area() / 255;
        double centerValue = Core.sumElems(center).val[1] / CENTER_ROI.area() / 255;
        double delta = leftValue - centerValue;

        left.release();
        center.release();


        telemetry.addData("[Pozitia]", "Valori detectate");


        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");

        telemetry.addData("delta value", delta);


        boolean posLeft=false;
        boolean posCenter=false;
        boolean posRight=false;

        if(Math.abs(delta)<=PERCENT_COLOR_THRESHOLD)
        {
            posRight = true;
        }

        if(delta < -0.3)
        {
            posLeft = true;
        }

        if(delta > 0.2)
        {
            posCenter = true;
        }


        if (posLeft) {
            location = Location.LEFT;
            telemetry.addData("Location", "left");
        }
        else
        if (posCenter)
        {
            location = Location.CENTER;
            telemetry.addData("Location", "center");
        }
        else
        {
            location = Location.RIGHT;
            telemetry.addData("Location", "right");
        }


        telemetry.update();

        Scalar red = new Scalar(0, 0, 255);
        Scalar green = new Scalar(0, 255, 0);

        Imgproc.rectangle(input, LEFT_ROI, posLeft==true?green:red,1);
        Imgproc.rectangle(input, CENTER_ROI, posCenter==true?green:red,1);

        return input;
    }

    public Location getLocation() {
        return location;
    }

}