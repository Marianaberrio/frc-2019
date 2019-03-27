/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;

/**
 * Arm Subsystem
 */
public class Arm extends Subsystem {
  private OI io = OI.getInstace();

  @Override
  public void initDefaultCommand() {
  }

  public void initPossition() {
    while (!io.limitSwitch.get()) {
      io.armTalon.set(ControlMode.PercentOutput, -0.2);
    }
  }

  public void setBallLevel(int level) {

  }

  public void setDiskLevel(int level) {

  }
}