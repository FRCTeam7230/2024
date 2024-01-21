import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class OrangeTorusDetection {

    // Function to load torus images from the "Assets" folder
    private static Mat[] loadTorusImages(String folderPath) {
        File folder = new File(folderPath);
        File[] files = folder.listFiles();
        Mat[] torusImages = new Mat[files.length];
        
        for (int i = 0; i < files.length; i++) {
            torusImages[i] = Imgcodecs.imread(files[i].getAbsolutePath());
        }
        
        return torusImages;
    }

    // Function for orange color detection
    private static Mat detectOrangeTorus(Mat frame, Scalar lowerOrange, Scalar upperOrange) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
        
        Mat mask = new Mat();
        Core.inRange(hsvFrame, lowerOrange, upperOrange, mask);
        
        Mat result = new Mat();
        Core.bitwise_and(frame, frame, result, mask);
        
        return result;
    }

    // Main function for live camera feed
    public static void main(String[] args) {
        // Set the path to the "Assets" folder (Made Universal)
        String folderPath = "./src/Assets";

        // Example range for orange color in HSV
        Scalar lowerOrange = new Scalar(0, 100, 100);
        Scalar upperOrange = new Scalar(20, 255, 255);

        // Load torus images
        Mat[] torusImages = loadTorusImages(folderPath);

        // Start CameraServer to handle camera input
        CameraServer cameraServer = CameraServer.getInstance();
        UsbCamera camera = cameraServer.startAutomaticCapture();
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
        
        CvSink cvSink = cameraServer.getVideo();
        CvSource outputStream = cameraServer.putVideo("Live Orange Torus Detection", 640, 480);

        // Mat to store camera frames
        Mat frame = new Mat();

        while (true) {
            // Grab a frame from the camera
            cvSink.grabFrame(frame);

            // Detect orange torus in the frame
            Mat result = detectOrangeTorus(frame, lowerOrange, upperOrange);

            // Send the result to the output stream
            outputStream.putFrame(result);
        }
    }
}
