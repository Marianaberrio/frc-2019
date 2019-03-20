/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements RobotMap {

  private static OI _io;
  // joystick.
  public Joystick driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
  // public Joystick asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);
  public WPI_TalonSRX leftFrontTalon, rightFrontTalon;
  public WPI_TalonSRX armTalon, handTalon;
  public DoubleSolenoid hanSolenoid, shooterSolenoid;
  public Compressor mainCompressor;
  public Encoder armEncoder;

  public static OI getInstace() {
    if (_io == null)
      _io = new OI();
    return _io;
  }

  private OI() {
    super();
    rightFrontTalon = new WPI_TalonSRX(TALON_DT_RIGHT_PORT);
    leftFrontTalon = new WPI_TalonSRX(TALON_DT_LEFT_PORT);
    armTalon = new WPI_TalonSRX(TALON_ARM_PORT);
    handTalon = new WPI_TalonSRX(TALON_HAND_PORT);

    driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
    // asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);

    mainCompressor = new Compressor();
    hanSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_HAND_FWD_PORT, SELENOID_DOUBLE_HAND_RVS_PORT);
    shooterSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_PUSHER_FWD_PORT, SELENOID_DOUBLE_PUSHER_RVS_PORT);

    armEncoder = new Encoder(ENCODER_ARM_CHANNEL_A, ENCODER_ARM_CHANNEL_B, false, Encoder.EncodingType.k4X);
  }

  public void initSetup() {
    initTalons();
    initEncoders();
    initNeumatics();
  }

  public void initTalons() {
    leftFrontTalon.configFactoryDefault();
    rightFrontTalon.configFactoryDefault();
    armTalon.configFactoryDefault();
    handTalon.configFactoryDefault();
  }

  public void initNeumatics() {
    mainCompressor.setClosedLoopControl(true);
    mainCompressor.start();
  }

  public void stopCompressor() {
    mainCompressor.stop();
  }

  public void initEncoders() {
    initArmEncoder();
  }

  public void initArmEncoder() {
    armEncoder.setMaxPeriod(0.1);
    armEncoder.setMinRate(10);
    armEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    armEncoder.setReverseDirection(false);
    armEncoder.setSamplesToAverage(7);

    armEncoder.reset();
  }
}
