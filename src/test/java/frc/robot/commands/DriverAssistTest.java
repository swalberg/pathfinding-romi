package frc.robot.commands;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import java.util.Optional;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class DriverAssistTest {

    @Test
    public void testFindPoseToLeftOfSeenReef_WithEmptyTransform() {
        // Mock the camera-to-target transform supplier
        Optional<Transform3d> cameraToTargetTransform = Optional.empty();

        // Mock the current pose supplier
        Pose2d currentPose = new Pose2d(2.0, 3.0, new Rotation2d());

        // Call the method
        Optional<Pose2d> result = DriverAssist.findPoseToLeftOfSeenReef(cameraToTargetTransform, currentPose);

        // Verify the result
        assertTrue(result.isEmpty(), "Result should be empty");
    }

    /*
     * Simplest case of the target being straight ahead. All we have to do is drive straight and compensate for 
     * both camera location and the depth of the center-to bumper
     */
    @Test
    public void testFindPoseToLeftOfSeenReef_WhenItIsStraightAhead() {
        // Mock the camera-to-target transform supplier
        Optional<Transform3d> cameraToTargetTransform = Optional.of(new Transform3d(0.0, 1.0, 0.0, new Rotation3d(new Rotation2d(Math.PI))));

        // Mock the current pose supplier
        Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d());

        // Call the method
        Optional<Pose2d> result = DriverAssist.findPoseToLeftOfSeenReef(cameraToTargetTransform, currentPose);

        // Verify the result
        assertTrue(result.isPresent(), "Result should be present");
        Pose2d expectedPose = new Pose2d(-0.15, 0.93, new Rotation2d()); // Expected pose after transformations
        assertEquals(expectedPose.getX(), result.get().getX(), 0.01, "X coordinate mismatch");
        assertEquals(expectedPose.getY(), result.get().getY(), 0.01, "Y coordinate mismatch");
        assertEquals(expectedPose.getRotation().getRadians(), result.get().getRotation().getRadians(), 0.01, "Rotation mismatch");
    }

}