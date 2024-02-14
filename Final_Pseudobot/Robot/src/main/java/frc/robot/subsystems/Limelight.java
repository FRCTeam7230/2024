package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
/* TODO:
 * - set up the general limelight [x (maybe? i dont know ill test with the limelight)]
 * - set up the apriltag detection []
 * - set up the red blue team integration [x (its just a skeleton right now ill implement apriltags into this with the limelight)]
 * - test this integration with the apriltags and the red blue team chooserand integration []
 */
public class Limelight {
    private static NetworkTableInstance table = null;
    public static double targetX, targetY, visionTargets;
    public static double targetArea;
    public static void limelight(String teamColor) {
        // this is where we set which pipeline to use based on the team color that gets passed here, the
        // rest of the code works off of this
        if (teamColor == "red") {
            // get red team pipeline
        } else if (teamColor == "blue") {
            // get blue team pipeline
        }
    }
    public static void refreshData() {
        // this is where we refresh the limelight data
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta"); (commented out for now, we might not need this)
        NetworkTableEntry tv = table.getEntry("tv");
        
        // read values periodically
        visionTargets = tv.getDouble(0.0);
        targetX = tx.getDouble(0.0);
        targetY = ty.getDouble(0.0);
        // targetArea = ta.getDouble(0.0); (commented out for now, we might not need this)
    }
    // public static double[] updateTarget(){
        // this refreshes what apriltags it should see (i hope)
    // }

    public static void manualLimelightOff(boolean manualLayout){
        if (manualLayout){
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
        }

    } // this is for when we want to manually turn off limelight

    public static void initializeLimelightOff(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
    } // this sets up limelight to be turned off

    public static double getTargetAngleX() {
        refreshData();
        return targetX;
    } // this gets the apriltag's x angle offset

    public static double getTargetAngleY(){
        refreshData();
        return targetY;
    } // this gets the apriltag's y angle offset

    //public static boolean proceedMoving(double objAreaRequired) {
        // refreshData();
        // this method's placement might be temporary, ill be working apriltag detection and will deem later if needed


}