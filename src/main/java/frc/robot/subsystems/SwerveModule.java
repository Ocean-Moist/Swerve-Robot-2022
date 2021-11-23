// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
    private static final double WHEEL_RADIUS = 0.0508;
    private static final int ENCODER_RESOLUTION = 4096;

    private static final double MODULE_MAX_ANGULAR_VELOCITY = Drivetrain.MAX_ANGULAR_SPEED;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final CANEncoder driveEncoder;
    private final DutyCycleEncoder turningEncoder;
    private final PIDController drivePIDController = new PIDController(1, 0, 0); // needs tuning

    private final ProfiledPIDController turningPIDController
            = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION)); // needs tuning

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3); // example gains (need to tune)
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5); // example gains (need to tune)

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel      CANSparkMax output for the drive motor.
     * @param turningMotorChannel    CANSparkMax output for the turning motor.
     * @param turningEncoderChannel  Input for the turning encoder channel
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel) {
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        this.driveEncoder = driveMotor.getEncoder();
        this.turningEncoder = new DutyCycleEncoder(turningEncoderChannel);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        driveEncoder.setMeasurementPeriod(4);
        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / 1.6);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        turningEncoder.setDistancePerRotation(2 * Math.PI / ENCODER_RESOLUTION);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.get()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedforwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                turningPIDController.calculate(turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforwardOutput =
                turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.set(driveOutput + driveFeedforwardOutput);
        turningMotor.set(turnOutput + turnFeedforwardOutput);
    }

    /**
     * Zeros all the SwerveModule encoders.
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.reset();
    }

}
