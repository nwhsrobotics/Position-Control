/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 NWHS Jagbots.                                           */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// This example shows how to use a Talon in Position control mode.
// It uses the SmartDashboard to allow tuning of the PID parameters.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

// Position Hold Subsystem
// Controls one motor in Position mode using PID parameters set through SmartDashboard.

public class PosHoldSubsystem extends Subsystem {
  // Some constants
  // Which CAN Id we'll use if dashboard doesn't specify one.
  static final int DEFAULT_TALON_ID = 1;

  // Encoder counts per revolution
  private static final int ENC_COUNT_PER_REV = 4096;

  // Talon have two sets of PID params, or slots, we use slot 0.
  private static final int SLOT_IDX = 0;

  // Talons also can run two PID loops, called PRIMARY and AUXILIARY.  For position control
  // only the PRIMARY loop is used.
  private static final int PID_PRIMARY = 0;

  // On certain talon operations, time out with an error after this period.
  private static final int TALON_TIMEOUT_MS = 100;

  private static final double DEFAULT_P = 1.0;
  private static final int DEFAULT_CURR_LIMIT = 2;
  private static final double DEFAULT_MAX_INTEGRAL = 2.0 * DEFAULT_P;

  // Our motor controller
  int m_canId = DEFAULT_TALON_ID;
  WPI_TalonSRX m_talon;

  // motor enabled flag
  boolean m_enabled = false; 
  
  // Position to hold (in encoder counts)
  int m_holdPosEnc = 0;

  // PID parameters and other Talon settings.
  double m_p = 0.0;
  double m_i = 0.0;
  double m_d = 0.0;
  double m_maxIntegral = 0.0;
  int m_maxAmps = 1;
  
  // Called from Robot.robotInit to init this subsystem.
  public void init() {
    // get parameters from smartdashboard
    readPreferences();

    m_talon = new WPI_TalonSRX(m_canId);

    // Select which set of PID parameter's we'll use.
    m_talon.selectProfileSlot(SLOT_IDX, PID_PRIMARY);

    // set sensor source and phase
    m_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_PRIMARY, TALON_TIMEOUT_MS);
    m_talon.setSensorPhase(false);

    configTalon();
  }

  @Override
  public void initDefaultCommand() {
    // Do nothing.
  }

  @Override
  public void periodic() {
    // Read operator input
    double holdPosDeg = Robot.m_oi.readPositionDeg();

    // Convert to encoder ticks
    m_holdPosEnc = degToEncoder(holdPosDeg);

    // Update motor output 
    if (!m_enabled) {
      m_talon.set(ControlMode.Disabled, 0.0);
    }
    else {
      m_talon.set(ControlMode.Position, m_holdPosEnc);
    }

    // Read Talon position and current
    double actualPosDeg = encToDeg(m_talon.getSelectedSensorPosition(PID_PRIMARY));
    double current = m_talon.getOutputCurrent();

    // update displayed position and current
    // On smartdashboard, add these indicators.
    SmartDashboard.putNumber("target", holdPosDeg);
    SmartDashboard.putNumber("actual", actualPosDeg);
    SmartDashboard.putNumber("hold current", current);
    SmartDashboard.putBoolean("hold active", m_enabled);
  }

  // Enable the motor in position hold mode
  // This method is called by PosHoldToggle Command, in response to A button.
  // When enabled, current motor position is assumed to be zero and 
  // integrator is cleared.  PID params are written to Talon and the
  // Talon is updated with the currently commanded position.
  public void enable(boolean enabled) {
    System.out.printf("setting enabled: %b\n", enabled);
    m_enabled = enabled;

    // set zero position when motor is enabled
    ErrorCode status = m_talon.setSelectedSensorPosition(0, PID_PRIMARY, TALON_TIMEOUT_MS);
    checkStatus(status, "setSelectedSensorPosition failed.");

    // Clear integrator
    status = m_talon.setIntegralAccumulator(0.0, PID_PRIMARY, TALON_TIMEOUT_MS);
    checkStatus(status, "Error clearing integral accumulator.");
  }

  // Is position hold enabled?
  public boolean isEnabled() {
    System.out.printf("Checking enabled: %b\n", m_enabled);
    return m_enabled;
  }

  // Update parameters from Preferences and write to Talon.
  public void updateParameters() {
    // Read PID params from Preferences
    readPreferences();

    // Write updated params to Talon
    configTalon();
  }

  // Read new PID params from Preferences
  // Preferences are stored on the RoboRIO and editable via the Dashboard.
  // This method is called from PosHoldUpdateParams Command, in response to B button.
  private void readPreferences() {
    Preferences prefs;

    prefs = Preferences.getInstance();

    // Read settings from SmartDashboard preferences control.
    m_canId = prefs.getInt("holdPos_canId", DEFAULT_TALON_ID);
    m_p = prefs.getDouble("holdPos_p", DEFAULT_P);
    m_i = prefs.getDouble("holdPos_i", 0.0);
    m_d = prefs.getDouble("holdPos_d", 0.0);
    m_maxAmps = prefs.getInt("holdPos_limit", DEFAULT_CURR_LIMIT);
    m_maxIntegral = prefs.getDouble("holdPos_maxIntegral", DEFAULT_MAX_INTEGRAL);

    // Print message to confirm update happened.
    System.out.printf("Updated params:\n");
    System.out.printf("    holdPos_canId: %d\n", m_canId);
    System.out.printf("    holdPos_p: %f\n", m_p);
  }

  // Write new PID params to Talon.
  private void configTalon() {
    ErrorCode status;

    // write PID values
    status = m_talon.config_kP(SLOT_IDX, m_p, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting P parameter.");

    status = m_talon.config_kI(SLOT_IDX, m_i, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting I parameter.");

    status = m_talon.config_kD(SLOT_IDX, m_d, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting P parameter.");

    // set max integrator accumulator
    m_talon.configMaxIntegralAccumulator(SLOT_IDX, m_maxIntegral, TALON_TIMEOUT_MS);

    // write current limit to talon.
    m_talon.configContinuousCurrentLimit(m_maxAmps, TALON_TIMEOUT_MS);
    m_talon.configPeakCurrentLimit(0, TALON_TIMEOUT_MS);
  }
  
  // Convert degrees to encoder units
  private int degToEncoder(double degrees) {
    return (int)(ENC_COUNT_PER_REV * degrees / 360.0); 
  }

  // Convert encoder units to degrees.
  private double encToDeg(int enc) {
    return (double)(enc * 360.0 / ENC_COUNT_PER_REV);
  }

  // Prints error messages when certain calls fail.
  private void checkStatus(ErrorCode status, String msg) {
    if (status != ErrorCode.OK) {
      System.out.printf("Error: %d, %s\n", status, msg);
    }
  }

}
