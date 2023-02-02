package org.firstinspires.ftc.teamcode.pipelines;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class pipelineFeatureTesting extends OpenCvPipeline {
    Telemetry telemetry;

    public pipelineFeatureTesting(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Bounds for detecting a specific color.
    // They are made public static because then they can be edited with EOCV-Sim.
    public static Scalar lowerBound1 = new Scalar(0, 0, 0, 0);
    public static Scalar upperBound1 = new Scalar(120, 255, 255, 255);


    // Used to store the input image shifted to a different color scheme.
    Mat colorShiftedIMG = new Mat();

    // A mask.
    Mat mask = new Mat();

    // An empty mat to be referenced when an empty mat is needed.
    // This avoids making new mats.
    Mat EMPTY_MAT = new Mat();


    // Finds and then returns the largest contour in a list of contours.
    public MatOfPoint findLargestContour(ArrayList <MatOfPoint> detectedCountours) {
        // return a blank MatOfPoint if there's no detected contours
        if (detectedCountours.isEmpty()) {
            return new MatOfPoint();
        }

        // the biggest contour size is defined as the first contour's width times height, or area
        int biggestContourSize = detectedCountours.get(0).width() * detectedCountours.get(0).height();
        // the biggest contour is defined as the first contour
        MatOfPoint biggestContour = detectedCountours.get(0);

        // runs for each contour, m
        for (MatOfPoint m:detectedCountours) {
            // the current area is the contour's width times height
            int currentArea = m.width() * m.height();
            // runs if the current area is larger than the previously recorded largest
            if (currentArea > biggestContourSize) {
                // sets the biggest contour size to be the current contour area
                biggestContourSize = currentArea;
                // sets sets the biggest contour d=to be the current contour
                biggestContour = m;
            }
        }

        // returns the biggest contour
        return biggestContour;
    }

    @Override
    public Mat processFrame(Mat input) {
        // input is the webcam image.

        // Converts the image's color from RGB to another color space.
        Imgproc.cvtColor(input, colorShiftedIMG, Imgproc.COLOR_RGB2BGR);
        // Blurs the image a little.
        Imgproc.medianBlur(colorShiftedIMG, colorShiftedIMG, 5);

        // A binary image of the contours and a bounding box over the largest one.
        mask = showBiggestContour(input, lowerBound1, upperBound1);

        // returns the sum of the masks
        return colorShiftedIMG;
    }

    public Mat showBiggestContour(Mat input, Scalar lowerBound, Scalar upperBound) {
        Mat mask = new Mat();
        ArrayList<MatOfPoint> detectedContours = new ArrayList<>();
        MatOfPoint biggestContour = new MatOfPoint();
        Rect boundingBox = new Rect();

        // See if pixels in input fall within the range of lowerBound to upperBound.
        // Store the binary image output in mask.
        Core.inRange(input, lowerBound, upperBound, mask);

        // Find the contours in image and store them in the list detectedContours.
        Imgproc.findContours(mask, detectedContours, EMPTY_MAT, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        // Finds and stores the biggest contour.
        biggestContour = findLargestContour(detectedContours);

        // Finds the bounding box for the biggest contour.
        boundingBox = Imgproc.boundingRect(biggestContour);

        // Draws the bounding box on the mask.
        Imgproc.rectangle(mask, boundingBox, new Scalar(255, 255, 255));

        // Returns the mask.
        return mask;
    }
}
