// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Vision;
import frc.robot.Constants.DriveConstants;

/**
 * The RomiDrivetrain class represents the drivetrain subsystem of the Romi robot.
 * It provides methods for controlling the robot's movement, including autonomous
 * and teleoperated driving, as well as managing odometry and sensor data.
 * 
 * <p>This class includes:
 * <ul>
 *   <li>Encoders for measuring wheel distances and speeds</li>
 *   <li>A gyro for measuring the robot's orientation</li>
 *   <li>PID controllers and feedforward for precise motor control</li>
 *   <li>Odometry for tracking the robot's position on the field</li>
 *   <li>Integration with PathPlanner for autonomous path following</li>
 * </ul>
 * 
 * <p>Key features:
 * <ul>
 *   <li>Robot-relative and field-relative driving capabilities</li>
 *   <li>Arcade drive for teleoperated control</li>
 *   <li>Real-time tuning of PID and feedforward parameters via SmartDashboard</li>
 *   <li>Visualization of robot pose and paths on a Field2d object</li>
 * </ul>
 * 
 * <p>Usage:
 * <ul>
 *   <li>Call {@link #arcadeDrive(double, double)} for teleoperated driving.</li>
 *   <li>Use {@link #driveRobotRelative(ChassisSpeeds)} for autonomous driving.</li>
 *   <li>Access odometry data with {@link #getCurrentPose()}.</li>
 *   <li>Reset encoders and odometry with {@link #resetEncoders()} and {@link #setCurrentPose(Pose2d)}.</li>
 * </ul>
 * 
 * <p>Note: This class implements {@link AutoCloseable} to ensure proper cleanup of hardware resources.
 */
public class RomiDrivetrain extends SubsystemBase implements AutoCloseable {
  private static final double MAX_SPEED = 0.5;
  private final RomiGyro gyro = new RomiGyro();
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.070;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark leftMotor = new Spark(0);
  private final Spark rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder leftEncoder = new Encoder(4, 5);
  private final Encoder rightEncoder = new Encoder(6, 7);

  private final PIDController leftPID = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD);
   private final PIDController rightPID = new PIDController(DriveConstants.kP, DriveConstants.kI,
      DriveConstants.kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV,
      DriveConstants.kA);

  // Kinematics helps us translate between the wheel speeds and the robot speeds
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.142);
  DifferentialDriveOdometry odometry;

  private Pose2d currentPose = new Pose2d();
  private Field2d field = new Field2d();

  private Vision vision;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // debugging the drive train PID
    SmartDashboard.putNumber("forwardPID/P", DriveConstants.kP);
    SmartDashboard.putNumber("forwardPID/D", DriveConstants.kD);
    SmartDashboard.putNumber("forwardPID/I", DriveConstants.kI);
    SmartDashboard.putNumber("feedforward/Ks", DriveConstants.kS);
    SmartDashboard.putNumber("feedforward/Kv", DriveConstants.kV);
    SmartDashboard.putNumber("feedforward/Ka", DriveConstants.kA);

    // Use meters as unit for encoder distances
    leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetEncoders();

    SmartDashboard.putData(field);
    odometry = new DifferentialDriveOdometry(getGyroAngle(), leftEncoder.getDistance(), rightEncoder.getDistance(),
        new Pose2d());
    // Invert right side since motor is flipped
    rightMotor.setInverted(true);

    vision = new Vision();


    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      field.getObject("path").setPoses(poses);
    });

    configureAutoBuilder();
  }

  private void configureAutoBuilder() {
    if (AutoBuilder.isConfigured()) {
      // AutoBuilder is already configured, no need to do it again
      return;
    }
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    AutoBuilder.configure(
        this::getCurrentPose, // Robot pose supplier
        this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPLTVController(0.02, MAX_SPEED), // PPLTVController is the built in path following controller for differential
        // drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Get the current robot speeds off the encoders
   * 
   * @return a ChassisSpeeds object representing the speed of the robot in meters
   *         per second
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    var leftSpeed = leftEncoder.getRate();
    var rightSpeed = rightEncoder.getRate();
    var speeds = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
    SmartDashboard.putNumber("getRobotRelativeSpeeds/x", speeds.vxMetersPerSecond);
    return speeds;
  }

  /**
   * Drive the robot using robot-relative speeds. This is useful for autonomous
   * driving where you
   * want to control the robot's speed and rotation rate.
   * 
   * @param speeds a ChassisSpeeds object representing the speed of the robot in
   *               meters per second
   */
  public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    final double fudgeFactor = 0.7;
    // concert the chassis speeds to wheel speeds
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // limit the wheel speeds to the max speed
    wheelSpeeds.leftMetersPerSecond = MathUtil.clamp(wheelSpeeds.leftMetersPerSecond * fudgeFactor, -MAX_SPEED, MAX_SPEED);
    wheelSpeeds.rightMetersPerSecond = MathUtil.clamp(wheelSpeeds.rightMetersPerSecond * fudgeFactor, -MAX_SPEED, MAX_SPEED);
    // set the wheel speeds
    setSpeeds(wheelSpeeds);
  }

  /**
   * Set the speeds of the left and right motors using PIDF control
   * 
   * @param speeds a DifferentialDriveWheelSpeeds object representing the speed of
   *               the left and right motors in meters per second
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
    final double leftOutput = leftPID.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPID.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    SmartDashboard.putNumber("setSpeeds/leftRate", leftEncoder.getRate());
    SmartDashboard.putNumber("setSpeeds/rightRate", rightEncoder.getRate());
    SmartDashboard.putNumber("setSpeeds/leftRequestedSpeed", speeds.leftMetersPerSecond);
    SmartDashboard.putNumber("setSpeeds/rightRequestedSpeed", speeds.rightMetersPerSecond);
    SmartDashboard.putNumber("setSpeeds/leftOutput", leftOutput);
    SmartDashboard.putNumber("setSpeeds/rightOutput", rightOutput);
    SmartDashboard.putNumber("setSpeeds/leftFeedforward", leftFeedforward);
    SmartDashboard.putNumber("setSpeeds/rightFeedforward", rightFeedforward);
    leftMotor.setVoltage(leftOutput + leftFeedforward);
    rightMotor.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drive the robot using arcade drive. This is useful for teleop driving where
   * you want to control
   * the robot's speed and rotation rate with a joystick
   * 
   * @param xaxisSpeed  the x-axis (forward) speed of the robot in motor % units
   *                    (-1 to 1)
   * @param zaxisRotate the rotation rate of the robot in motor % units (-1 to 1)
   */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    // TODO: If we're 100% forward and turning, does the robot actually turn?
    zaxisRotate = MathUtil.applyDeadband(zaxisRotate, 0.02);
    xaxisSpeed = MathUtil.applyDeadband(xaxisSpeed, 0.02);
    SmartDashboard.putNumber("arcadeDrive/xaxisSpeed", xaxisSpeed);
    SmartDashboard.putNumber("arcadeDrive/zaxisRotate", zaxisRotate);

    var speeds = kinematics
        .toWheelSpeeds(new ChassisSpeeds(xaxisSpeed * MAX_SPEED, 0, zaxisRotate));
    setSpeeds(speeds);
  }

  /**
   * Reset the encoders to zero.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Get the distance traveled by the left encoder in meters
   * 
   * @return the distance traveled by the left encoder in meters
   */
  public double getLeftDistanceMeters() {
    return leftEncoder.getDistance();
  }

  /**
   * Get the distance traveled by the right encoder in meters
   * 
   * @return the distance traveled by the right encoder in meters
   */
  public double getRightDistanceMeters() {
    return rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // Present the PID settings to SmartDashboard so we can tweak them
    leftPID.setP(SmartDashboard.getNumber("forwardPID/P", 0.0));
    leftPID.setD(SmartDashboard.getNumber("forwardPID/D", 0.0));
    leftPID.setI(SmartDashboard.getNumber("forwardPID/I", 0.0));
    rightPID.setP(SmartDashboard.getNumber("forwardPID/P", 0.0));
    rightPID.setD(SmartDashboard.getNumber("forwardPID/D", 0.0));
    rightPID.setI(SmartDashboard.getNumber("forwardPID/I", 0.0));

    feedforward.setKs(SmartDashboard.getNumber("feedforward/Ks", 0.0));
    feedforward.setKv(SmartDashboard.getNumber("feedforward/Kv", 0.0));
    feedforward.setKa(SmartDashboard.getNumber("feedforward/Ka", 0.0));

    // Get the rotation of the robot from the gyro.
    // Update the pose
    currentPose = odometry.update(getGyroAngle(),
        leftEncoder.getDistance(),
        rightEncoder.getDistance());
    // Update the Field2d object with the current pose
    field.setRobotPose(currentPose);
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getRate());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Optional<Transform3d> getTransformToTarget() {
    return vision.getTransformToTarget();
  }

  /**
   * Get the current pose of the robot
   * 
   * @return Pose2d object representing the current pose of the robot
   */
  public Pose2d getCurrentPose() {
    return currentPose;
  }

  public void setCurrentPose(Pose2d currentPose) {
    odometry.resetPosition(getGyroAngle(), getLeftDistanceMeters(), getRightDistanceMeters(), currentPose);
    this.currentPose = currentPose;
  }

  /**
   * Get the current angle of the robot. Counter-clockwise increasing
   * 
   * @return Rotation2d object representing the current angle of the robot
   */
  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  @Override
  public void close() {
    gyro.close();
    leftMotor.close();
    rightMotor.close();
    leftEncoder.close();
    rightEncoder.close(); 
  }
}
