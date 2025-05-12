package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriverAssist {
    private static final double BUMPER_TO_CENTER = 0.10;
    private static final double CENTER_TO_CAMERA = 0.03;
    public static Command driveToPose(Pose2d pose) {
        System.out.println("Driving to pose: " + pose);
        PathConstraints constraints = new PathConstraints(
            0.25, 0.25,
            3, Units.degreesToRadians(720));
    
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        );
    }

    /**
     * Drive to a pose, or do nothing if the pose is empty
     * @param pose - an Optional containing a Pose2d
     * @return a Command to drive
     */
    public static Command driveToPose(Optional<Pose2d> pose) {
        if (pose.isPresent()) {
            return driveToPose(pose.get());
        }
        return Commands.none();
    }

  public static Command driveToLeftOfSeenReef(Supplier<Optional<Transform3d>> cameraToTargetTransform, Supplier<Pose2d> currentPose) {
    return driveToPose(findPoseToLeftOfSeenReef(cameraToTargetTransform.get(), currentPose.get()));
  }
  /**
   * Drive to a place 6 inches to the left of the target that Photon sees.
   * This involves a lot of coordinate shifting. The robot has a current pose, which is where
   * it _thinks_ it is on the field, and there's where it actually is. 
   * 
   * Photon Vision returns a transform from the camera to the target, so what we'll do is apply that transform
   * and then shift it to the left by 6 inches.
   * @return an Optional containing a Pose2d object, relative to the robot's current pose
   */

   public static Optional<Pose2d> findPoseToLeftOfSeenReef(Optional<Transform3d> cameraToTargetTransform, Pose2d currentPose) {
    // Get the transform from the camera to the target
    if (cameraToTargetTransform.isPresent()) {
      // convert current pose to 3d so we can use the transform which is in 3d
      var currentIn3d = new Pose3d(currentPose);

      /* Current pose is relative to the center of the robot. The transform was relative to the camera.
       * The Robot has a non-zero area so we want to put the front bumper on the target.
       */
      
      // Update the transform to be relative to the center of the robot. Let's say the camera is 3cm ahead of the center (+Y)
      var centerToTargetTransform = cameraToTargetTransform.get().plus(new Transform3d(0.0, -CENTER_TO_CAMERA, 0, new Rotation3d()));
      
      // Take our current pose and transform it by the estimate from the camera to the target
      // This gives us the pose of the target in the robot's coordinate system
      var targetPose = currentIn3d.transformBy(centerToTargetTransform);

      // As the target is facing us, we want to flip it 180 degrees
      targetPose = targetPose.transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));

      // Now we want to move the robot 15cm to the left of the target, and back about 6cm to account for the bumper
      var onTheLeftReefPose = targetPose.transformBy(new Transform3d( -0.15, -BUMPER_TO_CENTER, 0, new Rotation3d()));

      // We want a 2d pose so we need to convert the 3d pose to a 2d pose
      var onTheLeftReefPose2d = new Pose2d(onTheLeftReefPose.getX(), onTheLeftReefPose.getY(), onTheLeftReefPose.getRotation().toRotation2d());
      return Optional.of(onTheLeftReefPose2d);
    } else {
      return Optional.empty();
    }
  }

}