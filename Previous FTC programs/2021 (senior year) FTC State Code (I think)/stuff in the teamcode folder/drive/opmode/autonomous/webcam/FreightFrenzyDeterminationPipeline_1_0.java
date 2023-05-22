package org.firstinspires.ftc.teamcode.drive.opmode.autonomous.webcam;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class FreightFrenzyDeterminationPipeline_1_0 extends OpenCvPipeline
{
    /*
     * An enum to define the Freight position
     */
    public enum FreightPosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    /*
     * Some color constants
     */
    //Color Vectors:
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
//    static final Scalar RED = new Scalar(255, 0, 0);
//    static final Scalar YELLOW = new Scalar(255, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,68);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(145,68);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(285,68);

//CHARLIE NOTE: the above points need to be changed once we know where the spots will be to our webcams, and the above distances need to be changed to make sure the areas are the eight size.

    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 15;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    //Just points, not regions:
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y); //A is same as top left)
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

//CHARLIE NOTE: the above region making code really should be put into a single funciton

    /*
     * Working variables
     */
//    Mat region1_G, region2_G, region3_G;

    //Initializing Matrices:
    //Regions:
    public Mat region1= new Mat();
    public Mat region2= new Mat();
    public Mat region3= new Mat();
    //Unused:
    public Mat customColorSpace = new Mat();
//    public Mat Cr = new Mat();
//    public Mat YCrCb = new Mat();
    //Whole image:
    public Mat WholeImage = new Mat();
    //Averages:
    int avg1, avg2, avg3;

    //From the enum Freight Position:
    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile FreightPosition position = FreightPosition.LEFT;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
//        void inputToCb(Mat input)
//        {
//            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 2);
//        }
//    public void inputToCr(Mat input){
//        Imgproc.cvtColor(input, customColorSpace, Imgproc.COLOR_RGB2HSV);
//        Mat red1 = new Mat();
//        Scalar bottom = new Scalar(0, 70, 50);
//        Scalar bottomEdge = new Scalar(10, 255, 255);
//        Core.inRange(customColorSpace, bottom, bottomEdge, red1);
//        Mat red2 = new Mat();
//        Scalar top = new Scalar(170, 70, 50);
//        Scalar topEdge = new Scalar(180, 255, 255);
//        Core.inRange(customColorSpace, top, topEdge, red2);
//        Core.bitwise_or(red1, red2, Cr);
//        // Make everything that's the color we're looking for turn white
//        // Core.bitwise_or(Cr, input, input);
//
//        // flip the pixels that we're seeing as "red" to yellow!
//        // also: Draw pixels by drawing a 1x1 rectangle is *masterful* code!
//        // TODO: This should use bitwise and/bitwise or to do this really...
//        Rect r = new Rect(new Point(0,0), new Point(1,1));
//        for (int i = 0; i < Cr.width(); i++) {
//            for (int j = 0; j < Cr.height(); j++) {
//                if (Cr.get(j, i)[0] > 0) {
//                    r.x = i;
//                    r.y = j;
//                    r.width = 1;
//                    r.height = 1;
//                    Imgproc.rectangle(input, r, YELLOW);
//                }
//            }
//        }
//    }
    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
//            inputToCr(firstFrame);
        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        WholeImage = firstFrame; //Sets the matrix to the matrix gotten by the webcam (image)
        region1 = WholeImage.submat(new Rect(region1_pointA, region1_pointB));
        region2 = WholeImage.submat(new Rect(region2_pointA, region2_pointB));
        region3 = WholeImage.submat(new Rect(region3_pointA, region3_pointB));

    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
//             inputToCr(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1).val[1];
        avg2 = (int) Core.mean(region2).val[1];
        avg3 = (int) Core.mean(region3).val[1];
//CHARLIE NOTE: I changed “Cb” to “G” in the above 3 lines because we are working with RGB instead of the other thing.
//I VERY MUCH doubt this will work, but I need to go and it gives an idea of what we need. Or we can just learn what the other thing is.
        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region3_pointA, // First point which defines the rectangle
                region3_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */
        int maxOneTwo = Math.max(avg1, avg2);
        int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
        if(max == avg1) // Was it from region 1?
        {
            position = FreightPosition.LEFT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg2) // Was it from region 2?
        {
            position = FreightPosition.CENTER; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }
        else if(max == avg3) // Was it from region 3?
        {
            position = FreightPosition.RIGHT; // Record our analysis

            /*
             * Draw a solid rectangle on top of the chosen region.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill
        }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public FreightPosition getAnalysis()

    {
        return position;
    }
//    public double getAvg1()
//
//    {
//        return avg1;
//    }
}