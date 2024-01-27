import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.shuffleboard.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class VisionRobot extends IterativeRobot {
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource outputStream;
    private MjpegServer mjpegServer;

    private ShuffleboardTab tab;
    private NetworkTableEntry lowHEntry;
    private NetworkTableEntry highHEntry;
    private NetworkTableEntry lowSEntry;
    private NetworkTableEntry highSEntry;
    private NetworkTableEntry lowVEntry;
    private NetworkTableEntry highVEntry;

    private Mat frame = new Mat();
    private Mat hsv = new Mat();
    private Mat threshold = new Mat();

    @Override
    public void robotInit() {
        camera = CameraServer.getInstance().startAutomaticCapture();
        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);

        tab = Shuffleboard.getTab("Vision");
        lowHEntry = tab.add("Low H", 0).getEntry();
        highHEntry = tab.add("High H", 180).getEntry();
        lowSEntry = tab.add("Low S", 0).getEntry();
        highSEntry = tab.add("High S", 255).getEntry();
        lowVEntry = tab.add("Low V", 0).getEntry();
        highVEntry = tab.add("High V", 255).getEntry();

        mjpegServer = new MjpegServer("Threshold Server", 1185);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        cvSink.grabFrame(frame);
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lowerBound = new Scalar(lowHEntry.getDouble(0), lowSEntry.getDouble(0), lowVEntry.getDouble(0));
        Scalar upperBound = new Scalar(highHEntry.getDouble(0), highSEntry.getDouble(0), highVEntry.getDouble(0));

        Core.inRange(hsv, lowerBound, upperBound, threshold);

        outputStream.putFrame(threshold);
        mjpegServer.getSource().putFrame(frame);
    }

    @Override
    public void testPeriodic() {
    }

    public static void main(String... args) {
        RobotBase.startRobot(VisionRobot::new);
    }
}
