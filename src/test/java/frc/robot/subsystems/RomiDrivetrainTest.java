package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class RomiDrivetrainTest {
  private RomiDrivetrain romiDrivetrain;
  private PWMSim leftMotorSim, rightMotorSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    leftMotorSim = new PWMSim(0);
    rightMotorSim = new PWMSim(1);
    romiDrivetrain = new RomiDrivetrain();
  }

  @AfterEach
  void teardown() {
    romiDrivetrain.close();
  }

  @Test
  public void testArcadeDriveZeroBand() {
    // Test the arcade drive with zero band actually uses zero band
    romiDrivetrain.arcadeDrive(0.0, 0.001);
    assertEquals(leftMotorSim.getSpeed(), 0);
    assertEquals(rightMotorSim.getSpeed(), 0);
  }
  @Test
  public void senseOfMotorsIsCorrect() {
    // driving forward, one should be going forward, the other backward
    romiDrivetrain.arcadeDrive(2.0, 0.0);
    assertTrue(leftMotorSim.getSpeed() > 0);
    assertTrue(rightMotorSim.getSpeed() < 0);
  }
}