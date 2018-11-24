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

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

// Position Hold Subsystem
// Controls one motor in Position mode using PID parameters set through SmartDashboard.

public class PosHoldSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Our motor controller
  static final int DEFAULT_TALON_ID = 1;
  int m_canId = DEFAULT_TALON_ID;
  TalonSRX m_talon;

  // Position to hold (in encoder counts)
  int m_holdPosEnc = 0;

  // PID parameters
  double m_p = 0.0;
  double m_i = 0.0;
  double m_d = 0.0;

  // motor enabled flag
  private boolean m_enabled = false;

  private double m_maxIntegral;
  private int m_maxAmps;

  private static final int ENC_COUNT_PER_REV = 4096;
  private static final int PID_IDX = 0;  // This example only uses PID parameter set 0.

  // On certain talon operations, time out with an error after this period.
  private static final int TALON_TIMEOUT_MS = 100;

  private static final double DEFAULT_P = 1.0;
  private static final int DEFAULT_CURR_LIMIT = 2;
  private static final double DEFAULT_MAX_INTEGRAL = 2.0 * DEFAULT_P;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new PosHoldOff());
  }

  public void init() {
    // get parameters from smartdashboard
    readDashboardParams();

    m_talon = new TalonSRX(m_canId);

    // set sensor source and phase
    m_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_IDX, TALON_TIMEOUT_MS);
    m_talon.setSensorPhase(false);

    configTalon();
  }

  @Override
  public void periodic() {
    double holdPosDeg = Robot.m_oi.readPositionDeg();

    // Convert to encoder ticks
    m_holdPosEnc = degToEncoder(holdPosDeg);

    // Update motor output 
    updatePosition();

    // update displayed position and current
    // On smartdashboard, add these indicators.
    SmartDashboard.putNumber("hold target", holdPosDeg);
    SmartDashboard.putNumber("hold position", getPositionDeg());
    SmartDashboard.putNumber("hold current", getCurrent());
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

    if (m_enabled) {
      // set zero position when motor is enabled
      setPositionDeg(0.0);
    }

    // Clear integrator
    ErrorCode status = m_talon.setIntegralAccumulator(0.0, PID_IDX, TALON_TIMEOUT_MS);
    if (status != ErrorCode.OK) {
      // Log error.  (Need better logging)
      System.out.println("Error clearing integral accumulator.\n");
    }

    // Make sure all configuration params are up to date.
    configTalon();

    // Command talon to desired position.
    updatePosition();
  }

  // Is position hold enabled?
  public boolean isEnabled() {
    System.out.printf("Checking enabled: %b\n", m_enabled);
    return m_enabled;
  }

  // Read new PID params from Smartdashboard
  // This method is called from PosHoldUpdateParams Command, in response to B button.
  public void readDashboardParams() {
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

  // set current position (typically used to zero the position)
  private void setPositionDeg(double positionDeg) {
    int posEnc = degToEncoder(positionDeg);

    ErrorCode status = m_talon.setSelectedSensorPosition(posEnc, PID_IDX, TALON_TIMEOUT_MS);
    if (status != ErrorCode.OK) {
      // Log error.  (Need better logging)
      System.out.println("Error setting sensor position.\n");
    }
  }

  // Convert degrees to encoder units
  private int degToEncoder(double degrees) {
    return (int)(ENC_COUNT_PER_REV * degrees / 360.0); 
  }

  // Convert encoder units to degrees.
  private double encToDeg(int enc) {
    return (double)(enc * 360.0 / ENC_COUNT_PER_REV);
  }

  // Read encoder position from Talon and return in degrees.
  private double getPositionDeg() {
    return encToDeg(m_talon.getSelectedSensorPosition(PID_IDX));
  }

     // Read current from Talon (in Amps)
  private double getCurrent() {
    return m_talon.getOutputCurrent();
  }

  // Called in Periodic, writes position to Talon
  private void updatePosition() {
    if (!m_enabled) {
      m_talon.set(ControlMode.Disabled, 0.0);
    }
    else {
      m_talon.set(ControlMode.Position, m_holdPosEnc);
    }
  }

  // Prints error messages when certain calls fail.
  private void checkStatus(ErrorCode status, String msg) {
    if (status != ErrorCode.OK) {
      System.out.println(msg);
    }
  }

  // Write new PID params to Talon.
  private void configTalon() {
    ErrorCode status;

    // write PID values
    status = m_talon.config_kP(PID_IDX, m_p, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting P parameter.");

    status = m_talon.config_kI(PID_IDX, m_i, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting I parameter.");

    status = m_talon.config_kD(PID_IDX, m_d, TALON_TIMEOUT_MS);
    checkStatus(status, "Error setting P parameter.");

    // set max integrator accumulator
    m_talon.configMaxIntegralAccumulator(PID_IDX, m_maxIntegral, TALON_TIMEOUT_MS);

    // write current limit to talon.
    m_talon.configContinuousCurrentLimit(m_maxAmps, TALON_TIMEOUT_MS);
    m_talon.configPeakCurrentLimit(0, TALON_TIMEOUT_MS);
  }
}
