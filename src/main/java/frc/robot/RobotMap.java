/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface RobotMap {
  // following variables to use with your drivetrain subsystem.
  // Talon ports
  public static final int TALON_DT_RIGHT_PORT = 5;
  public static final int TALON_DT_LEFT_PORT = 8;//
  public static final int TALON_ARM_PORT = 2;
  public static final int TALON_HAND_PORT = 3;

  // Neumatics
  public static final int PCM_01 = 0;
  public static final int COMPRESOR_MAIN_PORT = 0;
  public static final int SELENOID_DOUBLE_HAND_FWD_PORT = 2;
  public static final int SELENOID_DOUBLE_HAND_RVS_PORT = 4;
  public static final int SELENOID_DOUBLE_PUSHER_FWD_PORT = 0;
  public static final int SELENOID_DOUBLE_PUSHER_RVS_PORT = 1;

  // Analog devices
  public static final int LIMIT_SWITCH_ARM_PORT = 3;

  // Encoders
  public static final int ENCODER_ARM_CHANNEL_A = 0;
  public static final int ENCODER_ARM_CHANNEL_B = 1;

  // Joysticks
  public static final int JOYSTICK_MAIN_PORT = 0;
  public static final int JOYSTICK_SECUNDARY_PORT = 1;

  // Control BTNs
  public static final int LEFT_JOYSTICK = 1;
  public static int RIGHT_JOYSTICK = 5;
  // butttons
  public static int BTN_A_AXIS = 1;
  public static int BTN_B_AXIS = 2;
  public static int BTN_X_AXIS = 3;
  public static int BTN_Y_AXIS = 4;
  // back buttons R y L
  public static int BTN_L1_AXIS = 5;
  public static int BTN_R1_AXIS = 6;
  // back buttons R y L
  public static int BTN_BACK_AXIS = 7;
  public static int BTN_START_AXIS = 8;
  // back axis
  public static int BTN_L2_AXIS = 2;
  public static int BTN_R2_AXIS = 3;
  // Axis buttons
  public static int LEFT_BUTTON = 9;
  public static int RIGHT_BUTTON = 10;

}
