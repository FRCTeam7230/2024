package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.ArrayList;
import java.util.List;

public class Vision2Subsystem extends SubsystemBase {

    // Constants for color thresholding
    private final int MAX_VALUE_H = 180;
    private final int MAX_VALUE = 255;

    // Video capture object
    private VideoCapture capture;

    // UI components
    private JFrame frame;
    private JPanel controlsPanel;
    private JLabel rawLabel;
    private JLabel colorLabel;
    private JLabel bwLabel;
    private JLabel boxLabel;
    private JLabel distanceLabel;
    private JSlider lowHSlider;
    private JSlider highHSlider;
    private JSlider lowSSlider;
    private JSlider highSSlider;
    private JSlider lowVSlider;
    private JSlider highVSlider;

    public Vision2Subsystem() {
        // Initialize OpenCV
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Initialize UI
        initUI();

        // Initialize video capture
        capture = new VideoCapture();
        capture.open(0); // Adjust camera index as needed

        // Start capture thread
        Thread captureThread = new Thread(this::captureTask);
        captureThread.start();
    }

    private void initUI() {
        // Create frame
        frame = new JFrame("Color Thresholding");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                capture.release();
            }
        });

        // Create controls panel
        controlsPanel = new JPanel(new GridLayout(0, 1));

        // Create sliders
        lowHSlider = new JSlider(0, MAX_VALUE_H);
        highHSlider = new JSlider(0, MAX_VALUE_H);
        lowSSlider = new JSlider(0, MAX_VALUE);
        highSSlider = new JSlider(0, MAX_VALUE);
        lowVSlider = new JSlider(0, MAX_VALUE);
        highVSlider = new JSlider(0, MAX_VALUE);

        // Add sliders to the controls panel
        controlsPanel.add(new JLabel("Low Hue"));
        controlsPanel.add(lowHSlider);
        controlsPanel.add(new JLabel("High Hue"));
        controlsPanel.add(highHSlider);
        controlsPanel.add(new JLabel("Low Saturation"));
        controlsPanel.add(lowSSlider);
        controlsPanel.add(new JLabel("High Saturation"));
        controlsPanel.add(highSSlider);
        controlsPanel.add(new JLabel("Low Value"));
        controlsPanel.add(lowVSlider);
        controlsPanel.add(new JLabel("High Value"));
        controlsPanel.add(highVSlider);

        // Create image labels
        rawLabel = new JLabel();
        colorLabel = new JLabel();
        bwLabel = new JLabel();
        boxLabel = new JLabel();
        distanceLabel = new JLabel();

        // Add labels to the frame
        frame.add(controlsPanel, BorderLayout.WEST);
        frame.add(rawLabel, BorderLayout.NORTH);
        frame.add(colorLabel, BorderLayout.EAST);
        frame.add(bwLabel, BorderLayout.CENTER);
        frame.add(boxLabel, BorderLayout.SOUTH);
        frame.add(distanceLabel, BorderLayout.SOUTH);

        // Display the frame
        frame.pack();
        frame.setVisible(true);
    }

    private void captureTask() {
        while (true) {
            Mat frameMat = new Mat();
            capture.read(frameMat);

            // Convert BGR to HSV
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frameMat, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // Get slider values
            int lowH = lowHSlider.getValue();
            int highH = highHSlider.getValue();
            int lowS = lowSSlider.getValue();
            int highS = highSSlider.getValue();
            int lowV = lowVSlider.getValue();
            int highV = highVSlider.getValue();

            // Create mask
            Mat mask = new Mat();
            Core.inRange(hsvFrame, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), mask);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw bounding box
            Mat frameWithBox = frameMat.clone();
            frameWithBox = drawBoundingBox(frameWithBox, contours, new Scalar(0, 255, 0));

            // Calculate distance
            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                double apparentWidth = Imgproc.arcLength(new MatOfPoint2f(largestContour.toArray()), true);

                // Check if apparent width is not zero
                if (apparentWidth > 0) {
                    // Known physical width of the object in inches (example)
                    double knownWidthInches = 10;

                    // Known focal length of the camera (example, you need to calibrate this based on your camera)
                    double focalLength = 320.8;

                    // Estimate distance using triangulation
                    double distance = (knownWidthInches * focalLength) / apparentWidth;

                    // Display distance
                    distanceLabel.setText("Distance: " + String.format("%.2f", distance) + " inches");
                }
            }

            // Convert Mats to BufferedImages
            ImageIcon rawIcon = new ImageIcon(matToBufferedImage(frameMat));
            ImageIcon colorIcon = new ImageIcon(matToBufferedImage(mask));
            ImageIcon bwIcon = new ImageIcon(matToBufferedImage(hsvFrame));
            ImageIcon boxIcon = new ImageIcon(matToBufferedImage(frameWithBox));

            // Update labels
            rawLabel.setIcon(rawIcon);
            colorLabel.setIcon(colorIcon);
            bwLabel.setIcon(bwIcon);
            boxLabel.setIcon(boxIcon);

            // Sleep for a while
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private Mat drawBoundingBox(Mat frame, List<MatOfPoint> contours, Scalar color) {
        contours.sort((c1, c2) -> Double.compare(Imgproc.contourArea(c2), Imgproc.contourArea(c1)));

        if (!contours.isEmpty()) {
            Rect boundingRect = Imgproc.boundingRect(contours.get(0));
            Imgproc.rectangle(frame, boundingRect.tl(), boundingRect.br(), color, 2);
        }
        return frame;
    }

    private BufferedImage matToBufferedImage(Mat mat) {
        int type = BufferedImage.TYPE_BYTE_GRAY;
        if (mat.channels() > 1) {
            type = BufferedImage.TYPE_3BYTE_BGR;
        }
        BufferedImage image = new BufferedImage(mat.width(), mat.height(), type);
        mat.get(0, 0, ((DataBufferByte) image.getRaster().getDataBuffer()).getData());
        return image;
    }

    public static void main(String[] args) {
        // Load OpenCV
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Start camera server
        CameraServer.startAutomaticCapture();

    }
}