// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.romi.RomiGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiDrivetrain extends SubsystemBase {
  private final RomiGyro gyro = new RomiGyro();
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterMeters = 0.070;

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Kinematics helps us translate between the wheel speeds and the robot speeds
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.142);
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), new Pose2d());
  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private Pose2d currentPose = new Pose2d();
  private Field2d field = new Field2d();

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use meters as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeters) / kCountsPerRevolution);
    resetEncoders();

    SmartDashboard.putData(field);

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

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

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getCurrentPose, // Robot pose supplier
        this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
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
   * @return a ChassisSpeeds object representing the speed of the robot in meters per second
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    var leftSpeed = m_leftEncoder.getRate();
    var rightSpeed = m_rightEncoder.getRate();
    return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed));
  }

  /**
   * Drive the robot using robot-relative speeds. This is useful for autonomous
   * driving where you
   * want to control the robot's speed and rotation rate.
   * 
   * @param speeds a ChassisSpeeds object representing the speed of the robot in
   *               meters per second
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    m_diffDrive.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
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
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  @Override
  public void periodic() {
   // Get the rotation of the robot from the gyro.
   var gyroAngle = Rotation2d.fromDegrees(gyro.getAngle());
   // Update the pose
   currentPose = m_odometry.update(gyroAngle,
     m_leftEncoder.getDistance(),
     m_rightEncoder.getDistance());
    // Update the Field2d object with the current pose
    field.setRobotPose(currentPose);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static double getKcountsperrevolution() {
    return kCountsPerRevolution;
  }

  public static double getKwheeldiametermeters() {
    return kWheelDiameterMeters;
  }

  public Spark getM_leftMotor() {
    return m_leftMotor;
  }

  public Spark getM_rightMotor() {
    return m_rightMotor;
  }

  public Encoder getM_leftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getM_rightEncoder() {
    return m_rightEncoder;
  }

  public Pose2d getCurrentPose() {
    return currentPose;
  }

  public void setCurrentPose(Pose2d currentPose) {
    m_odometry.resetPosition(Rotation2d.fromDegrees(gyro.getAngle()), getLeftDistanceMeters(), getRightDistanceMeters(), currentPose);
    this.currentPose = currentPose;
  }
}
