package org.firstinspires.ftc.teamcode.drive.opmode.Meet1;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

public class SkystonePipeline extends OpenCvPipeline {

    final private static int frameHeight = 320;
    final private static int frameWidth = 240;

    public static int fromBottom = 20;
    public static int stoneHeight = 60;

    private int skystonePosition;

    final private static int rectHeight = frameHeight / 3;

    // defining stone detection zones
    Rect rectLeft = new Rect(fromBottom, 0, stoneHeight, rectHeight);
    Rect rectMiddle = new Rect(fromBottom, rectHeight, stoneHeight, rectHeight);
    Rect rectRight = new Rect(fromBottom, 2 * rectHeight, stoneHeight, rectHeight);

    @Override
    public Mat processFrame(Mat input) {

        Mat left = new Mat(input, rectLeft);
        Mat middle = new Mat(input, rectMiddle);
        Mat right = new Mat(input, rectRight);

        double leftValue = getValue(Core.mean(left));
        double middleValue = getValue(Core.mean(middle));
        double rightValue = getValue(Core.mean(right));

        // make an arrayList with the values of the 3 blocks for easier comparison
        ArrayList<Double> values = new ArrayList<>();
        values.add(leftValue);
        values.add(middleValue);
        values.add(rightValue);

        skystonePosition = values.indexOf(Collections.min(values));

        return input;
    }

    protected int getPosition() {
        return skystonePosition;
    }

    private double getValue(Scalar scalar) {
        double max = 0;

        for (int i = 0; i < 3; i++) {
            if (max < scalar.val[i]) {
                max = scalar.val[i];
            }
        }

        return max;
    }
}
