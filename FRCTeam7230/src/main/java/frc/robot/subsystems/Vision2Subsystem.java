package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.GripPipeline;

public class Vision2Subsystem {
    // Video capture object
    private VideoCapture capture;
    // private UsbCamera camera;
    // private CvSink cvSink;
    private CvSource output;
    // private CvSource maskOutput;

    private final int lowH = 0;
    private final int highH = 30;
    private final int lowS = 71;
    private final int highS = 255;
    private final int lowV = 151;
    private final int highV = 255;

    private GripPipeline pipeline = new GripPipeline();

    public Vision2Subsystem() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        capture = new VideoCapture();
        capture.open(0); // Adjust camera index as needed
        output = CameraServer.putVideo("Processed", 320, 240);
        // Start capture thread
    }

    public double[] getFromOffset(double yawCam, double disCam, double xDist, double yOffset, double xOffset) {

        double verticalDistance = Math.sqrt(Math.pow(disCam, 2) - Math.pow(xDist, 2));
        double robotyDistance = verticalDistance + yOffset;
        double robotxDistance = xDist + xOffset;

        double distRobot = Math.sqrt(Math.pow(robotxDistance, 2) + Math.pow(robotyDistance, 2));

        double yawRobot = Math.toDegrees(Math.atan(robotxDistance / robotyDistance));

        double[] data = { distRobot, yawRobot };

        return data;
    }

    public double[] captureTask(double xOffset, double yOffset) {

        Mat frameMat = new Mat();

        capture.read(frameMat);
        Mat frameWithBox = frameMat.clone();

        pipeline.process(frameMat);

        ArrayList<MatOfPoint> contours = pipeline.findContoursOutput();

        contours.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));
        frameWithBox = drawBoundingBox(frameWithBox, contours, new Scalar(0, 255, 0));
        output.putFrame(frameWithBox);
        double distance = 0;
        double yaw = 0;
        // double distance = 0;

        double datatosend = {};

        // Calculate distance
        if (!contours.isEmpty()) {
            Point topLeft = Imgproc.boundingRect(contours.get(0)).tl();
            Point bottomRight = Imgproc.boundingRect(contours.get(0)).br();

            double xPosition = (topLeft.x + (bottomRight.x - topLeft.x) / 2);
            double yPosition = (topLeft.y + (bottomRight.y - topLeft.y) / 2);

            double width = (bottomRight.x - topLeft.x);
            double height = (topLeft.y - bottomRight.y);

            SmartDashboard.putNumber("xPosition", xPosition);
            SmartDashboard.putNumber("yPosition", yPosition);
            SmartDashboard.putNumber("width", width);

            // // Check if apparent width is not zero
            if (width > 0) {

                double xPC = xPosition - 160;
                double yPC = yPosition - 120;

                double xPred = (xPC - 160) / 160;
                double yPred = (yPC - 120) / 120;

                double fovV = 0.73704695509;
                double fovH = 0.99954490199;

                double pitch = (yPred / 4) * fovV;
                yaw = (xPred / 4) * fovH;

                double heightCam = 24.94630774;
                double heightRing = 2.54;

                distance = -(heightRing - heightCam) / Math.tan(pitch);

                SmartDashboard.putNumber("pitch", Math.toDegrees(pitch));
                SmartDashboard.putNumber("yaw", Math.toDegrees(yaw));

                SmartDashboard.putNumber("distance", distance);
            }

            datatosend = getFromOffset(yaw, distance, xPosition, yOffset, xOffset);

            frameMat.release();
            frameWithBox.release();

            for (MatOfPoint p : contours) {
                p.release();
            }

        }
        return datatosend;
    }

    private Mat drawBoundingBox(Mat frame, List<MatOfPoint> contours, Scalar color) {
        if (!contours.isEmpty()) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(0));
            Imgproc.rectangle(frame, boundingRect.tl(), boundingRect.br(), color, 2);
        }
        return frame;
    }

    // private BufferedImage matToBufferedImage(Mat mat) {
    // int type = BufferedImage.TYPE_BYTE_GRAY;
    // if (mat.channels() > 1) {
    // type = BufferedImage.TYPE_3BYTE_BGR;
    // }
    // BufferedImage image = new BufferedImage(mat.width(), mat.height(), type);
    // mat.get(0, 0, ((DataBufferByte)
    // image.getRaster().getDataBuffer()).getData());
    // return image;
    // }

    // public static void main(String[] args) {
    // // Load OpenCV

    // // Start camera server
    // CameraServer.startAutomaticCapture();

}