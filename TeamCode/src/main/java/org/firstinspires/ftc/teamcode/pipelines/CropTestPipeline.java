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
import java.util.List;

public class CropTestPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public CropTestPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static int top = 0;
    public static int bottom = 100;
    public static int left = 0;
    public static int right = 100;
    Mat croppedIMG = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // WARNING: apparently this method creates a memory leak as both
        // rowRange and colRange create a new Mat every time they run.
        croppedIMG = input.rowRange(top, bottom);
        croppedIMG = croppedIMG.colRange(left, right);

        telemetry.addData("input size", input.size());
        telemetry.addData("cropped size", croppedIMG.size());
        telemetry.addData("top", top);
        telemetry.addData("bottom", bottom);
        telemetry.addData("left", left);
        telemetry.addData("right", right);
        telemetry.update();

        return croppedIMG;
    }
}
