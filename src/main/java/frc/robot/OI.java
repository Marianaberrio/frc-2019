/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements RobotMap {

  private static OI _io;
  // joystick.
  public Joystick driverJoystick;
  public WPI_TalonSRX leftFrontTalon, rightFrontTalon;
  public WPI_TalonSRX leftBackTalon, rightBackTalon;
  public WPI_TalonSRX armTalon;

  public static OI getInstace() {
    if (_io == null)
      _io = new OI();
    return _io;
  }

  private OI() {
    super();
    driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
    rightFrontTalon = new WPI_TalonSRX(TALON_DT_RIGHT_PORT);// 7
    leftFrontTalon = new WPI_TalonSRX(TALON_DT_LEFT_PORT);// 1
    rightBackTalon = new WPI_TalonSRX(TALON_DT_RIGHT_PORT);// 8
    leftBackTalon = new WPI_TalonSRX(TALON_DT_LEFT_PORT);// 2
    armTalon = new WPI_TalonSRX(TALON_ARM_PORT);


    driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
  }

  public void initSetup() {
    initTalons();
  }

  public void initTalons() {
    leftFrontTalon.configFactoryDefault();
    rightFrontTalon.configFactoryDefault();
    leftBackTalon.configFactoryDefault();
    rightBackTalon.configFactoryDefault();
    armTalon.configFactoryDefault();

    leftFrontTalon.setInverted(false);
    rightFrontTalon.setInverted(false);
  }
}
