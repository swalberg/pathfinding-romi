package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;

/** Add your docs here. */
public class Vision {
    private ArrayList <Camera> cameras = new ArrayList<>();

    public Vision() {
        cameras.add(new Camera("Camera1", new Pose3d()));
    }

}
