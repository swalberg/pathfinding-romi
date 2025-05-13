package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class DriverAssist {
    private static final double CENTER_TO_BUMPER = 0.10;
    private static final double CENTER_TO_CAMERA = 0.03;
    public static Command driveToPose(Pose2d pose) {
        System.out.println("Driving to pose: " + pose);
        PathConstraints constraints = new PathConstraints(
            0.25, 0.25,
            3, Units.degreesToRadians(720));
    
        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return new PrintCommand("starting").andThen(AutoBuilder.pathfindToPose(
            pose,
            constraints,
            edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
        )).andThen(new PrintCommand("done"));
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
    // Camera to target transform: Transform3d(Translation3d(X: 1.10, Y: 0.03, Z: 0.01), Rotation3d(Quaternion(0.004654719708315324, -0.19120064022869893, -0.0020357206761832942, -0.981537826373442)))
    // rather than run the camera , we'll just use a hardcoded transform
    Transform3d transform = new Transform3d(new Translation3d(1.10, 0.03, 0.01), new Rotation3d(new Quaternion(0.004654719708315324, -0.19120064022869893, -0.0020357206761832942, -0.981537826373442)));

    return driveToPose(findPoseToLeftOfSeenReef(Optional.of(transform), currentPose.get()));
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
       * The Robot has a non-zero area so we want to put the front bumper on the target.  */
      
       /* This code is not optimal. Some of the operations could be combined into one transform (but not all!)
        * Some of these operations are also composable... You can modify the transform or do multiple transforms
        * on the pose.
        */
      
      // Update the transform to be relative to the center of the robot. (-X)
      var centerToTargetTransform = cameraToTargetTransform.get().plus(new Transform3d(-CENTER_TO_CAMERA, 0, 0, new Rotation3d()));
      // Take our current pose and transform it by the estimate from the camera to the target
      // This gives us the pose of the target in the robot's coordinate system
      var targetPose = currentIn3d.transformBy(centerToTargetTransform);

      // Move forward by the distance from the center of the robot to the bumper (+X)
      targetPose = targetPose.transformBy(new Transform3d(CENTER_TO_BUMPER, 0, 0, new Rotation3d()));

      // As the target is facing us, we want to flip it 180 degrees so it's facing the same direction.
      // It's CCW positive but that doesn't matter because it's a complete 180
      targetPose = targetPose.transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));

      // And move 15cm to the left (+Y) to represent the reef
      targetPose = targetPose.transformBy(new Transform3d(0, 0.15, 0, new Rotation3d()));



      // We want a 2d pose so we need to convert the 3d pose to a 2d pose
      var onTheLeftReefPose2d = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().toRotation2d());
      return Optional.of(onTheLeftReefPose2d);
    } else {
      return Optional.empty();
    }
  }

}