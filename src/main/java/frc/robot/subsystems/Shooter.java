package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.rightShooterID);
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.leftShooterID);
  private double setpoint = 0;

  public Shooter() {
    m_rightShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);
    m_leftShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);
    m_rightShooter.setInverted(true);
    m_leftShooter.setControl(new Follower(ShooterConstants.rightShooterID, true));

//    SmartDashboard.putNumber("shooter p", 0);
//    SmartDashboard.putNumber("shooter d", 0);
//    SmartDashboard.putNumber("shooter voltage", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter actual speed", m_rightShooter.getRotorVelocity().getValue());
//    SmartDashboard.putNumber("shooter setpoint speed", setpoint);
//    SmartDashboard.putNumber("shooter setpoint speed", );

//    TalonFXConfiguration tempConfig = ShooterConstants.talonFXConfigs;
//    tempConfig.Slot0.kP = SmartDashboard.getNumber("shooter p", 0);
//    tempConfig.Slot0.kD = SmartDashboard.getNumber("shooter d", 0);
//    m_rightShooter.getConfigurator().apply(tempConfig);

//    setVoltage(SmartDashboard.getNumber("shooter voltage", 0));
  }

  public boolean atSetpoint() {
    return getVelocity() > (setpoint - ShooterConstants.SHOOTER_PID_TOLERANCE);
  }

  public void setSpeed(double speed) {
    setpoint = speed;
    m_rightShooter.setControl(new MotionMagicVelocityVoltage(speed));
  }

  public void setVoltage(double volts) {
    m_rightShooter.setControl(new VoltageOut(volts));
  }

  public double getVelocity() {
    return m_rightShooter.getVelocity().getValue();
  }
 
  public void stop() {
    m_rightShooter.stopMotor();
    m_leftShooter.stopMotor();
  }
}