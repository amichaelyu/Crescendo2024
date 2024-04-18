package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class PointWheelsAtCustom implements SwerveRequest {

    /**
     * The direction to point the modules toward.
     * This direction is still optimized to what the module was previously at.
     */
    public Rotation2d[] ModuleDirection = new Rotation2d[]{};
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

        for (int i = 0; i < modulesToApply.length; ++i) {
            SwerveModuleState state = new SwerveModuleState(0, ModuleDirection[i]);
            modulesToApply[i].apply(state, DriveRequestType, SteerRequestType);
        }

        return StatusCode.OK;
    }

    /**
     * Sets the direction to point the modules toward.
     * This direction is still optimized to what the module was previously at.
     *
     * @param moduleDirection Directions to point the modules toward
     * @return this request
     */
    public PointWheelsAtCustom withModuleDirection(Rotation2d[] moduleDirection) {
        this.ModuleDirection = moduleDirection;
        return this;
    }

    /**
     * Sets the type of control request to use for the drive motor.
     *
     * @param driveRequestType The type of control request to use for the drive motor
     * @return this request
     */
    public PointWheelsAtCustom withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    /**
     * Sets the type of control request to use for the steer motor.
     *
     * @param steerRequestType The type of control request to use for the steer motor
     * @return this request
     */
    public PointWheelsAtCustom withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }
}