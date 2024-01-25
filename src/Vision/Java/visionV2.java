import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class Robot extends TimedRobot {
    private UsbCamera camera;
    private Mat hsvImage;
    private Mat mask;
    private Mat output;

    private final Scalar lowerOrange = new Scalar(0, 150, 100);
    private final Scalar upperOrange = new Scalar(10, 255, 255);

    private final double knownWidth = 5.0; // Known physical width of the torus in centimeters (example)
    private final double focalLength = 1000.0; // Known focal length of the camera (example, you need to calibrate this based on your camera)

    @Override
    public void robotInit() {
        camera = CameraServer.getInstance().startAutomaticCapture();
        hsvImage = new Mat();
        mask = new Mat();
        output = new Mat();
    }

    @Override
    public void robotPeriodic() {
        // Perform image processing and distance estimation here

        // Example: Display distance on SmartDashboard
        SmartDashboard.putNumber("Distance", estimateDistance());
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    private double estimateDistance() {
        // Perform distance estimation based on image processing results
        // Use WPILib camera API to get the current frame from the camera
        CvSink cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Output", 640, 480);

        cvSink.grabFrame(output);

        // Convert the output Mat to a MatOfKeyPoint for further processing if needed
        MatOfKeyPoint keyPoints = new MatOfKeyPoint();
        // Example: Imgproc.goodFeaturesToTrack(output, keyPoints, ...);

        // Perform distance estimation logic here using WPILib classes and methods
        double distance = 0.0;
        // Example: distance = (knownWidth * focalLength) / apparentWidth;

        return distance;
    }

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
