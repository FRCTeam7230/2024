// // OUR GOAL IS TO MOVE THIS INTO VISONSUBSYSTEM.JAVA

// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode;
// import edu.wpi.first.cscore.VideoSink;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import org.opencv.core.*;

// import javax.swing.*;
// import java.awt.*;
// import java.awt.image.BufferedImage;
// import java.util.Map;

// public class VisionRobot extends TimedRobot {

//     private Scalar lowHsv, highHsv;

//     private ShuffleboardTab visionTab;

//     public static void main(String[] args) {
//         RobotBase.startRobot(VisionRobot::new);
//     }

//     @Override
//     public void robotInit() {
//         System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

//         lowHsv = new Scalar(0, 0, 0);
//         highHsv = new Scalar(180, 255, 255);

//         camera = CameraServer.startAutomaticCapture();
//         camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, WIDTH, HEIGHT, 30);

//         cvSink = CameraServer.getVideo();
//         outputStream = CameraServer.putVideo("Processed", WIDTH, HEIGHT);
//         matFrame = new Mat();

//         visionTab = Shuffleboard.getTab("Vision");

//         new Thread(() -> {
//             while (!Thread.interrupted()) {
//                 processFrame();
//             }
//         }).start();
//     }

//     private void processFrame() {
//         cvSink.grabFrame(matFrame);

//         // Perform image processing here
//         Mat processedFrame = performImageProcessing(matFrame);

//         // Display processed frame on Shuffleboard
//         displayOnShuffleboard(processedFrame);

//         outputStream.putFrame(processedFrame);
//     }

//     private Mat performImageProcessing(Mat frame) {
//         Mat hsvFrame = new Mat();
//         Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

//         // Get slider values (replace with your own GUI sliders)
//         int lowH = 0;
//         int highH = 180;
//         int lowS = 0;
//         int highS = 255;
//         int lowV = 0;
//         int highV = 255;

//         // Color Only View
//         Mat mask = new Mat();
//         Core.inRange(hsvFrame, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), mask);
//         Mat colorOnlyView = new Mat();
//         Core.bitwise_and(frame, frame, colorOnlyView, mask);

//         // Binarize
//         Mat bwFrame = new Mat();
//         Imgproc.cvtColor(colorOnlyView, bwFrame, Imgproc.COLOR_BGR2GRAY);
//         Imgproc.threshold(bwFrame, bwFrame, 128, 255, Imgproc.THRESH_BINARY);

//         // Draw bounding box
//         List<MatOfPoint> contours = new ArrayList<>();
//         Imgproc.findContours(bwFrame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

//         Mat frameWithBox = new Mat();
//         frameWithBox = drawBoundingBox(frame, contours, new Scalar(0, 255, 0));

//         return frameWithBox;
//     }

//     private Mat drawBoundingBox(Mat frame, List<MatOfPoint> contours, Scalar color) {
//         contours.sort(Comparator.comparingDouble(Imgproc::contourArea).reversed());

//         if (!contours.isEmpty()) {
//             MatOfPoint largestContour = contours.get(0);

//             Rect boundingRect = Imgproc.boundingRect(largestContour);
//             Imgproc.rectangle(frame, boundingRect.tl(), boundingRect.br(), color, 2);
//         }

//         return frame;
//     }

//     private void displayOnShuffleboard(Mat imageMat) {
//         BufferedImage image = convertMatToBufferedImage(imageMat);

//         // Display processed frame on Shuffleboard
//         visionTab.add("Processed Image", new ImageWidgetBuilder()
//                 .withImage(image)
//                 .withProperties(Map.of("Width", 3, "Height", 2))
//                 .build());
//     }

//     private BufferedImage convertMatToBufferedImage(Mat mat) {
//         int type = BufferedImage.TYPE_BYTE_GRAY;
//         if (mat.channels() > 1) {
//             type = BufferedImage.TYPE_3BYTE_BGR;
//         }
//         int bufferSize = mat.channels() * mat.cols() * mat.rows();
//         byte[] bytes = new byte[bufferSize];
//         mat.get(0, 0, bytes);

//         BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
//         image.getRaster().setDataElements(0, 0, mat.cols(), mat.rows(), bytes);

//         return image;
//     }

//     @Override
//     public void teleopInit() {
//         // Teleop initialization code
//     }

//     @Override
//     public void teleopPeriodic() {
//         // Teleop periodic code
//     }
// }
