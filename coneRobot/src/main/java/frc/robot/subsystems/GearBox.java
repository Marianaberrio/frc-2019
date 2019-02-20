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
public class GearBox extends Subsystem {

  DoubleSolenoid gear = OI.getInstace().gearChangeSolenoid;

  @Override
  public void initDefaultCommand() {
  }

  public void changeGear() {
    if (isHeavyGearEnable()) {
      setSpeedGear();
    } else {
      setHeavy();
    }
  }

  public void setHeavy() {
    if (isSpeedGearEnable())
      gear.set(Value.kForward);
  }

  public void setSpeedGear() {
    if (isHeavyGearEnable())
      gear.set(Value.kReverse);
  }

  public boolean isSpeedGearEnable() {
    return gear.get() == Value.kReverse;
  }

  public boolean isHeavyGearEnable() {
    return gear.get() == Value.kForward;
  }
}
