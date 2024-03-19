package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlipperConstants;

public class Flipper extends SubsystemBase {
    private final DigitalInput limitSwitchBottom = new DigitalInput(FlipperConstants.BOTTOM_SWITCH_ID);
    private final DigitalInput limitSwitchTop = new DigitalInput(FlipperConstants.TOP_SWITCH_ID);
    private final TalonSRX flipperMotor = new TalonSRX(FlipperConstants.MOTOR_ID);

    private final static Flipper INSTANCE = new Flipper();

    public static Flipper getInstance() {
        return INSTANCE;
    }

    private Flipper() {
        flipperMotor.setInverted(false);
        flipperMotor.setNeutralMode(NeutralMode.Brake);
        flipperMotor.configContinuousCurrentLimit(12);
        flipperMotor.configPeakCurrentLimit(12);

        SmartDashboard.putBoolean("Flipper At Top", atTop());
        SmartDashboard.putBoolean("Flipper At Bottom", atBottom());
    }

    public void dutyCycle(double input) {
        flipperMotor.set(ControlMode.PercentOutput, input);
    }

    public void flipUp() {
        if (!atTop()) {
            flipperMotor.set(ControlMode.PercentOutput, 0.3);
        }
    }

    public void flipDown() {
        if (!atBottom()) {
            flipperMotor.set(ControlMode.PercentOutput, -0.3);
        }
    }

    public void stop() {
        flipperMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean atTop() {
        return !limitSwitchTop.get();
    }

    public boolean atBottom() {
        return !limitSwitchBottom.get();
    }
}