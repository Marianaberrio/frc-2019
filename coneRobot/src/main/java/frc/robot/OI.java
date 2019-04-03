/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI implements RobotMap {

  private static OI _io;
  //// joystick.
  public Joystick driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
  public Joystick asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);
  public SpeedControllerGroup speedControllerDriveRight, speedControllerDriveLeft;
  public WPI_TalonSRX leftFrontTalon, rightFrontTalon;
  public WPI_TalonSRX leftSlaveTalon, rightSlaveTalon;
  public WPI_TalonSRX armTalon, handTalon;
  public DigitalInput limitSwitchArm;
  public DoubleSolenoid elevateRobotSolenoid, raptorWheelSolenoid, hanSolenoid, changeGearSolenoid;
  public Compressor mainCompressor;
  public Encoder armEncoder, driveRightEncoder, driveLeftEncoder;
  public AnalogInput frontSonar;
  public DifferentialDrive drive;
  // subsystems
  public Claw hand;
  public Arm arm;

  public static OI getInstace() {
    if (_io == null)
      _io = new OI();
    return _io;
  }

  private OI() {
    super();
    rightFrontTalon = new WPI_TalonSRX(TALON_DT_RIGHT_FRONT_PORT);
    leftFrontTalon = new WPI_TalonSRX(TALON_DT_LEFT_FRONT_PORT);
    rightSlaveTalon = new WPI_TalonSRX(TALON_DT_RIGHT_BACK_PORT);
    leftSlaveTalon = new WPI_TalonSRX(TALON_DT_LEFT_BACK_PORT);
    armTalon = new WPI_TalonSRX(TALON_ARM_PORT);
    handTalon = new WPI_TalonSRX(TALON_HAND_PORT);

    driverJoystick = new Joystick(JOYSTICK_MAIN_PORT);
    asistantJoystick = new Joystick(JOYSTICK_SECUNDARY_PORT);

    limitSwitchArm = new DigitalInput(LIMIT_SWITCH_ARM_PORT);
    frontSonar = new AnalogInput(0);
    frontSonar.setAccumulatorInitialValue(ULTRASONIC_SONAR_PORT);
    mainCompressor = new Compressor(COMPRESOR_MAIN_PORT);

    elevateRobotSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_ELEVATE_FWD_PORT, SELENOID_DOUBLE_ELEVATE_RVS_PORT);
    raptorWheelSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_WHEELS_FWD_PORT, SELENOID_DOUBLE_WHEELS_RVS_PORT);
    changeGearSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_GEAR_FWD_PORT, SELENOID_DOUBLE_GEAR_RVS_PORT);
    hanSolenoid = new DoubleSolenoid(SELENOID_DOUBLE_HAND_FWD_PORT, SELENOID_DOUBLE_HAND_RVS_PORT);

    armEncoder = new Encoder(ENCODER_ARM_CHANNEL_A, ENCODER_ARM_CHANNEL_B, false, Encoder.EncodingType.k4X);
    driveRightEncoder = new Encoder(ENCODER_DT_RIGHT_CHANNEL_A, ENCODER_DT_RIGHT_CHANNEL_B, false,
        Encoder.EncodingType.k4X);
    driveLeftEncoder = new Encoder(ENCODER_DT_LEFT_CHANNEL_A, ENCODER_DT_LEFT_CHANNEL_B, false,
        Encoder.EncodingType.k4X);
  }

  public void initSetup() {
    initTalons();
    initDriveTrain();
    initEncoders();
    initNeumatics();
    //autonomousInit();
    // initSubSystems();
  }

  public void initSubSystems() {
    hand = new Claw();
    arm = new Arm();
  }

  public void initDriveTrain() {
    speedControllerDriveRight = new SpeedControllerGroup(rightFrontTalon, rightSlaveTalon);
    speedControllerDriveLeft = new SpeedControllerGroup(leftFrontTalon, leftSlaveTalon);
    drive = new DifferentialDrive(speedControllerDriveLeft, speedControllerDriveRight);
  }

  public void initTalons() {
    leftFrontTalon.configFactoryDefault();
    leftSlaveTalon.configFactoryDefault();
    rightFrontTalon.configFactoryDefault();
    rightSlaveTalon.configFactoryDefault();
    armTalon.configFactoryDefault();
    handTalon.configFactoryDefault();
  }

  public void initNeumatics() {
    mainCompressor.setClosedLoopControl(true);
  }

  public void initEncoders() {
    initArmEncoder();
    initDTLeftEncoder();
    initDTRightEncoder();
  }

  public void initArmEncoder() {
    armEncoder.setMaxPeriod(0.1);
    armEncoder.setMinRate(10);
    armEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    armEncoder.setReverseDirection(false);
    armEncoder.setSamplesToAverage(7);

    armEncoder.reset();
  }

  public void initDTRightEncoder() {
    driveLeftEncoder.setMaxPeriod(0.1);
    driveLeftEncoder.setMinRate(10);
    driveLeftEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    driveLeftEncoder.setReverseDirection(false);
    driveLeftEncoder.setSamplesToAverage(7);

    driveLeftEncoder.reset();
  }

  public void initDTLeftEncoder() {
    driveRightEncoder.setMaxPeriod(0.1);
    driveRightEncoder.setMinRate(10);
    driveRightEncoder.setDistancePerPulse((Math.PI * (8.0 / 12.0)) / 7.0);
    driveRightEncoder.setReverseDirection(false);
    driveRightEncoder.setSamplesToAverage(7);

    driveRightEncoder.reset();
  }

  public void initAnalogInputs() {

  }
}
