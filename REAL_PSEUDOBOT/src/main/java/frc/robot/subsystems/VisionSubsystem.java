// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Rect;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static final int WIDTH = 640;
  private static final int HEIGHT = 480;

  private UsbCamera camera;
  private CvSink cvSink;
  private CvSource outputStream;
  private Mat matFrame;

  private Scalar lowHsv, highHsv;
  private ShuffleboardTab visionTab;

  public VisionSubsystem() {
    // this is init
  }

  public void robotInit() {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    lowHsv = new Scalar(0, 0, 0);
    highHsv = new Scalar(180, 255, 255);

    camera = CameraServer.startAutomaticCapture();
    camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, WIDTH, HEIGHT, 30);

    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Processed", WIDTH, HEIGHT);
    matFrame = new Mat();

    visionTab = Shuffleboard.getTab("Vision");

    new Thread(() -> {
      while (!Thread.interrupted()) {
        processFrame();
      }
    }).start();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  private void processFrame() {
    cvSink.grabFrame(matFrame);

    Mat processedFrame = performImageProcessing(matFrame);

    displayOnShuffleboard(processedFrame);

    outputStream.putFrame(processedFrame);
  }

  private Mat performImageProcessing(Mat frame) {
    Mat hsvFrame = new Mat();
    Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

    int lowH = 0;
    int highH = 180;
    int lowS = 0;
    int highS = 255;
    int lowV = 0;
    int highV = 255;

    // Color Only View
    Mat mask = new Mat();
    Core.inRange(hsvFrame, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), mask);
    Mat colorOnlyView = new Mat();
    Core.bitwise_and(frame, frame, colorOnlyView, mask);

    // Binarize
    Mat bwFrame = new Mat();
    Imgproc.cvtColor(colorOnlyView, bwFrame, Imgproc.COLOR_BGR2GRAY);
    Imgproc.threshold(bwFrame, bwFrame, 128, 255, Imgproc.THRESH_BINARY);

    // Draw bounding box
    List<MatOfPoint> contours = new ArrayList<>();
    Imgproc.findContours(bwFrame, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

    Mat frameWithBox = new Mat();
    frameWithBox = drawBoundingBox(frame, contours, new Scalar(0, 255, 0));

    return frameWithBox;
  }

  private Mat drawBoundingBox(Mat frame, List<MatOfPoint> contours, Scalar color) {
    // contours.sort(Comparator.comparingDouble(Imgproc::contourArea).reversed());

    if (!contours.isEmpty()) {
      MatOfPoint largestContour = contours.get(0);

      Rect boundingRect = Imgproc.boundingRect(largestContour);
      Imgproc.rectangle(frame, boundingRect.tl(), boundingRect.br(), color, 2);
    }

    return frame;
  }

  private void displayOnShuffleboard(Mat imageMat) {
    BufferedImage image = convertMatToBufferedImage(imageMat);

    JLabel picLabel = new JLabel(new ImageIcon(image));

    JPanel jPanel = new JPanel();
    jPanel.add(picLabel);

    JFrame f = new JFrame();
    f.setSize(new Dimension(picLabel.getWidth(), picLabel.getHeight()));
    f.add(jPanel);
    f.setVisible(true);

    // Display processed frame on Shuffleboard
    // visionTab.add("Processed Image", new ImageIcon()
    //     .withImage(image)
    //     .withProperties(Map.of("Width", 3, "Height", 2))
    //     .build());
  }

  private BufferedImage convertMatToBufferedImage(Mat mat) {
    int type = BufferedImage.TYPE_BYTE_GRAY;
    if (mat.channels() > 1) {
      type = BufferedImage.TYPE_3BYTE_BGR;
    }
    int bufferSize = mat.channels() * mat.cols() * mat.rows();
    byte[] bytes = new byte[bufferSize];
    mat.get(0, 0, bytes);

    BufferedImage image = new BufferedImage(mat.cols(), mat.rows(), type);
    image.getRaster().setDataElements(0, 0, mat.cols(), mat.rows(), bytes);

    return image;
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
