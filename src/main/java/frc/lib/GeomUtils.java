package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class GeomUtils {
    public static double distance(Pose2d pose1, Pose2d pose2) {
        Pose2d relPose = pose1.relativeTo(pose2);
        return Math.sqrt(Math.pow(relPose.getX(), 2) + Math.pow(relPose.getY(), 2));
    }
}