/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
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
  public Joystick asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);
  public WPI_TalonSRX leftFrontTalon, rightFrontTalon;
  public WPI_TalonSRX armTalon, handTalon;
  public DoubleSolenoid hanSolenoid, shooterSolenoid;
  public Compressor mainCompressor;
  public Encoder armEncoder, leftEncoder, rightEncoder;
  public DigitalInput limitSwitch;

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
    asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);

    mainCompressor = new Compressor();
    hanSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_HAND_FWD_PORT, SELENOID_DOUBLE_HAND_RVS_PORT);
    shooterSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_PUSHER_FWD_PORT, SELENOID_DOUBLE_PUSHER_RVS_PORT);

    armEncoder = new Encoder(ENCODER_ARM_CHANNEL_A, ENCODER_ARM_CHANNEL_B, false, Encoder.EncodingType.k4X);
    leftEncoder = new Encoder(ENCODER_LEFT_CHANNEL_A, ENCODER_LEFT_CHANNEL_B, false, Encoder.EncodingType.k4X);
    rightEncoder = new Encoder(ENCODER_RIGHT_CHANNEL_A, ENCODER_RIGHT_CHANNEL_B, true, Encoder.EncodingType.k4X);
    limitSwitch = new DigitalInput(3);
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
    initLeftWheelEncoder();
    initRightWheelEncoder();
  }

  public void initArmEncoder() {
    armEncoder.setMaxPeriod(0.1);
    armEncoder.setMinRate(10);
    armEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    armEncoder.setReverseDirection(false);
    armEncoder.setSamplesToAverage(7);

    armEncoder.reset();
  }
  public void initLeftWheelEncoder() {
    leftEncoder.setMaxPeriod(0.1);
    leftEncoder.setMinRate(10);
    leftEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    leftEncoder.setReverseDirection(false);
    leftEncoder.setSamplesToAverage(7);

    leftEncoder.reset();
  }
  public void initRightWheelEncoder() {
    rightEncoder.setMaxPeriod(0.1);
    rightEncoder.setMinRate(10);
    rightEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    rightEncoder.setReverseDirection(false);
    rightEncoder.setSamplesToAverage(7);

    rightEncoder.reset();
  }
}
