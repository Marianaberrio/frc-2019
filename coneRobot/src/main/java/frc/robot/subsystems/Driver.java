/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.OI;
import frc.robot.commands.DriveWithJoysticks;

/**
 * Add your docs here.
 */
public class Driver extends Subsystem {
  DifferentialDrive drive = OI.getInstace().drive;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoysticks());
  }

  public void stop() {
    drive.stopMotor();
  }

  public void straight(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void turn(double speed, double direction) {
    drive.curvatureDrive(speed, -direction, false);
  }

  public void rotate(double direction) {
    drive.tankDrive(-direction, direction);
  }

  public void tank(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }
}
