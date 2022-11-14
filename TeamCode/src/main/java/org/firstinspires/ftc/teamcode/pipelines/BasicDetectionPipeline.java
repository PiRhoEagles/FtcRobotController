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
import java.util.Arrays;

public class BasicDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public BasicDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // the bounds for detecting a specific color
    // they are made public static because then they can be edited with EOCV-Sim
    public static Scalar lowerBound1 = new Scalar(144.5, 15.6, 39.7, 0);
    public static Scalar upperBound1 = new Scalar(151.6, 52.4, 123.3, 255);

    public static Scalar lowerBound2 = new Scalar(222.4, 140.3, 0, 0);
    public static Scalar upperBound2 = new Scalar(247.9, 154.4, 2.8, 255);

    public static Scalar lowerBound3 = new Scalar(102, 233.8, 208.3, 0);
    public static Scalar upperBound3 = new Scalar(247.9, 238, 216.8, 255);

    // the states that can be detected
    public enum detectionStates {
        ONE,
        TWO,
        THREE
    }

    // declares the current detected state to ONE
    detectionStates state = detectionStates.ONE;

    // an array of the matrices to release
    private ArrayList<Mat> matsToRelease = new ArrayList<>();

    // releases all the memory leaks
    public void releaseMemoryLeaks() {
        // runs for every mat in matsToRelease
        for (Mat m: matsToRelease) {
            // releases the current mat, m
            m.release();
        }
    }

    // adds all these variables to the list of mats to release
    public void roundupMemory(Mat... Mats) {
        matsToRelease.addAll(Arrays.asList(Mats));
    }

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
        // input is the webcam image

        // releases the memory leaks from the previous loop
        releaseMemoryLeaks();

        // used to store the color shifted input
        Mat colorShiftedIMG = new Mat();

        // used to store different processed versions of the input
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask3 = new Mat();

        // converts the image's color from RGB to the YCrCb color space
        Imgproc.cvtColor(input, colorShiftedIMG, Imgproc.COLOR_RGB2YCrCb);
        // blurs the image a little
        Imgproc.medianBlur(colorShiftedIMG, colorShiftedIMG, 5);

        // gets exampleMat, a color shifted version of the input image, and converts it to
        // a binary image based on whether each pixel is within a certain range
        // this binary output image is stored as mask{num} for each corresponding range
        Core.inRange(colorShiftedIMG, lowerBound1, upperBound1, mask1);
        Core.inRange(colorShiftedIMG, lowerBound2, upperBound2, mask2);
        Core.inRange(colorShiftedIMG, lowerBound3, upperBound3, mask3);

        // detects the contours in mask{num} and stores them in the array detectedContours{num}
        ArrayList<MatOfPoint> detectedContours1 = new ArrayList<>();
        Imgproc.findContours(mask1,detectedContours1,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        ArrayList<MatOfPoint> detectedContours2 = new ArrayList<>();
        Imgproc.findContours(mask2,detectedContours2,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
        ArrayList<MatOfPoint> detectedContours3 = new ArrayList<>();
        Imgproc.findContours(mask3,detectedContours3,new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);

        // finds and stores the biggest contour in each detectedContours
        MatOfPoint biggestContour1 = findLargestContour(detectedContours1);
        MatOfPoint biggestContour2 = findLargestContour(detectedContours2);
        MatOfPoint biggestContour3 = findLargestContour(detectedContours3);

        // adds all the masks together so they can all display at the same time
        Mat sum = new Mat();
        Core.add(mask1, mask2, sum);
        Core.add(mask3, sum, sum);

        // creates and draws a bounding box around the largest contour in each mask
        Rect boundingBox1 = Imgproc.boundingRect(biggestContour1);
        Imgproc.rectangle(sum, boundingBox1, new Scalar(255, 255, 255));
        Rect boundingBox2 = Imgproc.boundingRect(biggestContour2);
        Imgproc.rectangle(sum, boundingBox2, new Scalar(255, 255, 255));
        Rect boundingBox3 = Imgproc.boundingRect(biggestContour3);
        Imgproc.rectangle(sum, boundingBox3, new Scalar(255, 255, 255));

        // resets the state to be ONE
        state = detectionStates.ONE;

        // changes the state from ONE to TWO if boundingBox2 is greater than bB1
        if (boundingBox2.area() > boundingBox1.area()) {
            state = detectionStates.TWO;
        }

        // changes the state to THREE if bB3 is bigger than bB1 and bB2
        if (boundingBox3.area() > boundingBox2.area() && boundingBox3.area() > boundingBox1.area()) {
            state = detectionStates.THREE;
        }

        // displays the detected state in telemetry
        //telemetry.addData("Detected State", state);
        //telemetry.update();

        // adds these variables to the array of things to release
        roundupMemory(mask1, mask2, mask3, colorShiftedIMG);

        // returns the sum of the masks
        return sum;
    }

    // can be called get the get the current state
    public detectionStates getState() {
        return state;
    }
}
