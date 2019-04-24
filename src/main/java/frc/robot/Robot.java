/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * TEAM FORCE 4707 PRACTICE ROBOT
 */
public class Robot extends TimedRobot implements RobotMap {
  private static OI oi;

  private DifferentialDrive drive;
  private SpeedControllerGroup leftSpeedController, rigthSpeedController;

  private static final double MAX_VELOCITY = 0.7;

  @Override
  public void robotInit() {
    oi = OI.getInstace();
    oi.initSetup();
    leftSpeedController = new SpeedControllerGroup(oi.leftFrontTalon, oi.leftBackTalon);
    rigthSpeedController = new SpeedControllerGroup(oi.rightFrontTalon, oi.rightBackTalon);
    drive = new DifferentialDrive(leftSpeedController, rigthSpeedController);
  }

  @Override
  public void teleopPeriodic() {
    double velocity = oi.driverJoystick.getRawAxis(1) * MAX_VELOCITY;
    double rotation = oi.driverJoystick.getRawAxis(4) * MAX_VELOCITY;
    smoothDrive(velocity, rotation);

    if (oi.driverJoystick.getRawButtonPressed(BTN_L1_AXIS)) {
      moveHand(0.7);
    } else if (oi.driverJoystick.getRawButtonReleased(BTN_L1_AXIS)) {
      moveHand(0.0);
    }


    
    if (oi.driverJoystick.getRawButtonPressed(BTN_R1_AXIS)) {
      moveHand(-0.7);
    } else if ((oi.driverJoystick.getRawButtonReleased(BTN_R1_AXIS))) {
      moveHand(0.0);
    }
    

    if (oi.driverJoystick.getRawButtonPressed(BTN_L1_AXIS)) {
      moveHand(0.7);
    } else if (oi.driverJoystick.getRawButtonReleased(BTN_L1_AXIS)) {
      moveHand(0.0);
    }
  }

  private void smoothDrive(double velocity, double rotation) {
    if (Math.abs(velocity) < 0.05) {
      velocity = 0;
    }
    if (Math.abs(rotation) < 0.05) {
      rotation = 0;
    }

    if (velocity != 0d && rotation == 0d) {
      drive.tankDrive(velocity, velocity);
    } else {
      if (rotation != 0d && velocity == 0d) {
        drive.tankDrive(-rotation, rotation);
      } else {
        drive.curvatureDrive(velocity, -rotation, false);
      }
    }
  }
  private void moveHand(double direction) {
    oi.armTalon.configFactoryDefault();
}
}
