/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;

/**
 * Shoother Subsystem
 */
public class Shooter extends Subsystem {
  Solenoid hand = OI.getInstace().shooterSolenoid;

  @Override
  public void initDefaultCommand() {
  }

  public void shot() {
    if (!hand.get())
      hand.set(true);
  }

  public void reload() {
    if (hand.get())
      hand.set(false);
  }
}
