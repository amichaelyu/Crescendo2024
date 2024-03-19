package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  private final TalonFX m_rightShooter = new TalonFX(ShooterConstants.rightShooterID, Constants.CAN_BUS_NAME);
  private final TalonFX m_leftShooter = new TalonFX(ShooterConstants.leftShooterID, Constants.CAN_BUS_NAME);
  private double setpoint = 0;
  private boolean stopped = false;

  private static final Shooter INSTANCE = new Shooter();

  public static Shooter getInstance() {
    return INSTANCE;
  }

  private Shooter() {
    m_rightShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);
    m_leftShooter.getConfigurator().apply(ShooterConstants.talonFXConfigs);

    m_rightShooter.setInverted(true);
    m_leftShooter.setInverted(false);

//    m_leftShooter.setControl(new Follower(ShooterConstants.rightShooterID, true));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter actual speed", m_rightShooter.getRotorVelocity().getValue());
    SmartDashboard.putBoolean("shooter at setpoint", atSetpoint());

//    if (Swerve.getInstance().getPose().getY() > Speaker.centerSpeakerOpening.getY()) {
//      rightSpin = 0.9;
//      leftSpin = 1.0;
//    }
//    else {
//      rightSpin = 1.0;
//      leftSpin = 0.9;
//    }
  }

  public boolean atSetpoint() {
    return getVelocity() > (setpoint - ShooterConstants.SHOOTER_PID_TOLERANCE);
  }

  public void setSpeed(double speed) {
    m_rightShooter.setControl(new MotionMagicVelocityVoltage(speed));
    m_leftShooter.setControl(new MotionMagicVelocityVoltage(speed));
  }

  public void setVoltage(double volts) {
    m_rightShooter.setControl(new VoltageOut(volts));
    m_leftShooter.setControl(new VoltageOut(volts));
  }

  public double getPosition() {
    return m_rightShooter.getPosition().getValue();
  }

  public double getVelocity() {
    return m_rightShooter.getVelocity().getValue();
  }

  public double getVoltage() {
    return m_rightShooter.getMotorVoltage().getValue();
  }

  public void stop() {
    setVoltage(0);
  }
}