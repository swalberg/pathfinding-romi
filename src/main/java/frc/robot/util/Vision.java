package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class Vision {
    Camera camera;

    public Vision() {
        camera = new Camera("Global_Shutter_Camera", new Pose3d());
    }

    public Optional<Transform3d> getTransformToTarget() {
        var target = camera.getBestResult();
        if (target.isPresent()) {
            var transform = target.get().getBestCameraToTarget();
            return Optional.of(transform);
        } else {
            return Optional.empty();
        }
    }

}
