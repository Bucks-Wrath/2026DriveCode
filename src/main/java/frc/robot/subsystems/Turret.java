package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Constants;
import frc.robot.DeviceIds;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = Constants.Shooter.Turret.MinimumTurretPosition;
	private double maxUpTravelPosition = Constants.Shooter.Turret.MaximumHoodPosition;

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = homePosition;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;
	public double shooterAddValue;

	private final static double onTargetThreshold = 0.25;
		
	private TalonFX turretKraken = new TalonFX(DeviceIds.Turret.LeadMotorId, "canivore");
	private Pigeon2 pigeon = new Pigeon2(12);

    private TalonFXConfiguration turretFXConfig = new TalonFXConfiguration();

	public Turret() {
		// Clear Sticky Faults
		this.turretKraken.clearStickyFaults();
		
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		turretFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		turretFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretFXConfig.CurrentLimits.StatorCurrentLimit = 35;

        /* PID Config */
        turretFXConfig.Slot0.kP = 0.2;
        turretFXConfig.Slot0.kI = 0;
        turretFXConfig.Slot0.kD = 0.01;

        /* Open and Closed Loop Ramping */
        turretFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        turretFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        turretFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        turretFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        turretFXConfig.MotionMagic.withMotionMagicAcceleration(300);
        turretFXConfig.MotionMagic.withMotionMagicCruiseVelocity(300);

        // Config Motor
        turretKraken.getConfigurator().apply(turretFXConfig);
        turretKraken.getConfigurator().setPosition(0.0);
	}

	public void positionControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.turretKraken.setControl(targetPositionDutyCycle);
	}

	public void positionManualControl(double targetPosition) {
		this.targetPosition = targetPosition;
	}

	public double getCurrentPosition() {
		return this.turretKraken.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.turretKraken.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isturretSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetturretEncoder() {
        try {
			turretKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Turret.resetTurretEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double joystickTurret(){
		double value = 0;
		value = -Robot.m_robotContainer.getOperatorLeftStickX();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Turret Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Turret Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Turret Position Error", this.getPositionError());
		SmartDashboard.putNumber("Turret Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Turret Current", this.getCurrentDraw());
		SmartDashboard.putNumber("Chassis Angle", getYaw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.turretKraken.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}

	public double getYaw() {
		double angle = pigeon.getYaw().getValueAsDouble();
		return angle;
    }
}   
