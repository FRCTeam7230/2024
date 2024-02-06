import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.ArrayList;
import java.util.List;

public class VisionRobot extends IterativeRobot {
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource outputStream;
    private Mat source;
    private Mat output;
    private JFrame frame;
    private JLabel imgRawLabel;
    private JLabel imgColorLabel;
    private JLabel imgBwLabel;
    private JLabel imgBoxLabel;

    @Override
    public void robotInit() {
        // Set up camera
        camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Processed", 320, 240);

        // Initialize matrices
        source = new Mat();
        output = new Mat();

        // Set up GUI
        SwingUtilities.invokeLater(this::createAndShowGUI);
    }

    private void createAndShowGUI() {
        frame = new JFrame("FRC Vision Processing");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        imgRawLabel = new JLabel();
        imgColorLabel = new JLabel();
        imgBwLabel = new JLabel();
        imgBoxLabel = new JLabel();

        JPanel viewsPanel = new JPanel(new GridLayout(2, 2));
        viewsPanel.add(imgRawLabel);
        viewsPanel.add(imgColorLabel);
        viewsPanel.add(imgBwLabel);
        viewsPanel.add(imgBoxLabel);

        frame.getContentPane().add(viewsPanel);
        frame.pack();
        frame.setVisible(true);
    }

    @Override
    public void teleopPeriodic() {
        // Retrieve the latest frame from the camera
        cvSink.grabFrame(source);

        // Process the frame
        processFrame(source, output);

        // Send the processed frame to the dashboard
        outputStream.putFrame(output);

        // Display the processed frames in the GUI
        updateImageLabels(source, output);
    }

    private void processFrame(Mat input, Mat output) {
        // Additional processing logic:
        // - Convert the image to grayscale
        Imgproc.cvtColor(input, output, Imgproc.COLOR_BGR2GRAY);

        // - Apply additional image processing steps based on your requirements
        //   For example, you can add color thresholding, contour detection, etc.
        //   Replace the following example with your actual processing logic

        // Example: Apply GaussianBlur
        Imgproc.GaussianBlur(output, output, new Size(5, 5), 0);

        // Example: Apply Canny edge detection
        Imgproc.Canny(output, output, 50, 150);

        // Example: Add bounding box
        addBoundingBox(output);

        // Add more processing steps as needed
    }

    private void addBoundingBox(Mat frame) {
        // Example: Add bounding box around the largest contour
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(frame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.stream().max(Comparator.comparing(Imgproc::contourArea)).orElse(null);

            if (largestContour != null) {
                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(frame, boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2);
            }
        }
    }

    private void updateImageLabels(Mat raw, Mat processed) {
        // Convert OpenCV Mat to BufferedImage for display in the GUI
        BufferedImage rawImage = matToBufferedImage(raw);
        BufferedImage processedImage = matToBufferedImage(processed);

        // Set images to labels
        imgRawLabel.setIcon(new ImageIcon(rawImage));
        imgColorLabel.setIcon(new ImageIcon(processedImage));
        imgBwLabel.setIcon(new ImageIcon(rawImage));  // Display the original image for the black and white view
        imgBoxLabel.setIcon(new ImageIcon(processedImage));
    }

    private BufferedImage matToBufferedImage(Mat mat) {
        // Convert OpenCV Mat to BufferedImage
        int type = BufferedImage.TYPE_BYTE_GRAY;
        if (mat.channels() > 1) {
            type = BufferedImage.TYPE_3BYTE_BGR;
        }
        int bufferSize = mat.channels() * mat.cols() * mat.rows();
        byte[] bytes = new byte[bufferSize];
        mat.get(0, 0, bytes);

        BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
        final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
        System.arraycopy(bytes, 0, targetPixels, 0, bytes.length);

        return image;
    }
}
