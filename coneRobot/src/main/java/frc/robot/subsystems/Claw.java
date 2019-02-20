/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;

/**
 * Hand movement subsystem
 */
public class Claw extends Subsystem {

  DoubleSolenoid hand = OI.getInstace().hanSolenoid;

  @Override
  public void initDefaultCommand() {
  }

  public void open() {
    if (isClawClose())
      hand.set(Value.kForward);
  }

  public void close() {
    if (isClawOpen())
      hand.set(Value.kReverse);
  }

  public boolean isClawOpen() {
    return hand.get() == Value.kReverse;
  }

  public boolean isClawClose() {
    return hand.get() == Value.kForward;
  }
}
