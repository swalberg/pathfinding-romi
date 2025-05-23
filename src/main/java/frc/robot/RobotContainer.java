// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriverAssist;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements AutoCloseable {
  // The robot's subsystems and commands are defined here...
  private final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  private final Joystick joystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_romiDrivetrain.setDefaultCommand(
        new RunCommand(() -> m_romiDrivetrain.arcadeDrive(-joystick.getY(), -joystick.getX()), m_romiDrivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand(() -> m_romiDrivetrain.setCurrentPose(new Pose2d())).andThen(DriverAssist.driveToPose(new Pose2d(1.0, 0.0, new Rotation2d())));
    //return (DriverAssist.driveToLeftOfSeenReef(m_romiDrivetrain::getTransformToTarget, m_romiDrivetrain::getCurrentPose));

  }

  @Override
  public void close() {
    // Close the joystick and any other resources
    m_romiDrivetrain.close();
  }
}
