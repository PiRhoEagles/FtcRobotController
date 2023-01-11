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

public class PowerPlayPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public PowerPlayPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // the bounds for detecting a specific color
    // they are made public static because then they can be edited with EOCV-Sim
    public static Scalar lowerBound1 = new Scalar(151.6, 216.8, 192.7, 0);
    public static Scalar upperBound1 = new Scalar(192.7, 255.0, 240.8, 255);

    public static Scalar lowerBound2 = new Scalar(242.3, 211.1, 117.6, 0);
    public static Scalar upperBound2 = new Scalar(255.0, 255.0, 185.6, 255);

    public static Scalar lowerBound3 = new Scalar(206.8, 155.8, 182.8, 0);
    public static Scalar upperBound3 = new Scalar(255.0, 188.4, 229.5, 255);


    // the states that can be detected
    public enum detectionStates {
        ONE,
        TWO,
        THREE
    }

    // declares the current detected state to ONE
    detectionStates state = detectionStates.ONE;

    // used to store the color shifted input
    Mat colorShiftedIMG = new Mat();

    // used to store different processed versions of the input
    Mat mask1 = new Mat();
    Mat mask2 = new Mat();
    Mat mask3 = new Mat();

    // used to store the detected contours in a mask
    ArrayList<MatOfPoint> detectedContours1 = new ArrayList<>();
    ArrayList<MatOfPoint> detectedContours2 = new ArrayList<>();
    ArrayList<MatOfPoint> detectedContours3 = new ArrayList<>();

    // used to store the biggest contour in each mask
    MatOfPoint biggestContour1 = new MatOfPoint();
    MatOfPoint biggestContour2 = new MatOfPoint();
    MatOfPoint biggestContour3 = new MatOfPoint();

    // used to store a sum of the masks
    Mat sum = new Mat();

    // used to store the bounding boxes for the largest contour in each mask
    Rect boundingBox1 = new Rect();
    Rect boundingBox2 = new Rect();
    Rect boundingBox3 = new Rect();

    // and empty mat
    Mat EMPTY_MAT = new Mat();

    // color of the bounding boxes
    Scalar BOUNDING_BOX_COLOR = new Scalar(255, 255, 255);


    // finds and then returns the largest contour
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

        // Empties some variables.
        // Some variables don't need to be emptied.
        //colorShiftedIMG = new Mat();
        //mask1 = new Mat();
        //mask2 = new Mat();
        //mask3 = new Mat();
        detectedContours1 = new ArrayList<>();
        detectedContours2 = new ArrayList<>();
        detectedContours3 = new ArrayList<>();
        //biggestContour1 = new MatOfPoint();
        //biggestContour2 = new MatOfPoint();
        //biggestContour3 = new MatOfPoint();
        //boundingBox1 = new Rect();
        //boundingBox2 = new Rect();
        //boundingBox3 = new Rect();
        //sum = new Mat();

        // converts the image's color from RGB to another color space
        Imgproc.cvtColor(input, colorShiftedIMG, Imgproc.COLOR_RGB2BGR);
        // blurs the image a little
        Imgproc.medianBlur(colorShiftedIMG, colorShiftedIMG, 5);

        // Gets colorShiftedIMG, a color shifted version of the input image, and converts it to
        // a binary image based on whether each pixel is within a certain range.
        // This binary output image is stored as mask{num} for each corresponding range.
        Core.inRange(colorShiftedIMG, lowerBound1, upperBound1, mask1);
        Core.inRange(colorShiftedIMG, lowerBound2, upperBound2, mask2);
        Core.inRange(colorShiftedIMG, lowerBound3, upperBound3, mask3);

        // detects the contours in mask{num} and stores them in the array detectedContours{num}
        Imgproc.findContours(mask1,detectedContours1, EMPTY_MAT, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(mask2,detectedContours2, EMPTY_MAT, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(mask3,detectedContours3, EMPTY_MAT, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        // finds and stores the biggest contour in each detectedContours
        biggestContour1 = findLargestContour(detectedContours1);
        biggestContour2 = findLargestContour(detectedContours2);
        biggestContour3 = findLargestContour(detectedContours3);

        // finds the bounding box for each biggest contour
        boundingBox1 = Imgproc.boundingRect(biggestContour1);
        boundingBox2 = Imgproc.boundingRect(biggestContour2);
        boundingBox3 = Imgproc.boundingRect(biggestContour3);

        // adds all the masks together so they can all display at the same time
        Core.add(mask1, mask2, sum);
        Core.add(sum, mask3, sum);

        // draws each bounding box on sum
        Imgproc.rectangle(sum, boundingBox1, BOUNDING_BOX_COLOR);
        Imgproc.rectangle(sum, boundingBox2, BOUNDING_BOX_COLOR);
        Imgproc.rectangle(sum, boundingBox3, BOUNDING_BOX_COLOR);

        // resets the state to be ONE
        state = detectionStates.ONE;

        // changes the state from ONE to TWO if bB2 is greater than bB1
        if (boundingBox2.area() > boundingBox1.area()) {
            state = detectionStates.TWO;
        }

        // changes the state to THREE if bB3 is bigger than bB1 and bB2
        if (boundingBox3.area() > boundingBox2.area() && boundingBox3.area() > boundingBox1.area()) {
            state = detectionStates.THREE;
        }

        // displays the detected state in telemetry
        // WARNING: remove this if this class is being used in another file
        //telemetry.addData("Detected State", state);
        //telemetry.update();

        // returns the sum of the masks
        return sum;
    }

    // can be called get the get the current state
    public detectionStates getState() {
        return state;
    }
}
