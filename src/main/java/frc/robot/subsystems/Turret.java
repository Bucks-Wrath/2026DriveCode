package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceIds;

public class Turret extends SubsystemBase {

	private TalonFX TurretKraken = new TalonFX(DeviceIds.Turret.LeadMotorId);
    private TalonFXConfiguration TurretFXConfig = new TalonFXConfiguration();
    private TalonFX TurretKrakenFollower = new TalonFX(DeviceIds.Turret.FollowerMotorId);


	public Turret() {
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		TurretFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        TurretFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Set Followers
		TurretKrakenFollower.setControl(new Follower(TurretKraken.getDeviceID(), null));

        /* Current Limiting */
        //TurretFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //TurretFXConfig.CurrentLimits.SupplyCurrentLimit = 20;
        //TurretFXConfig.CurrentLimits.SupplyCurrentThreshold = 30;
        //TurretFXConfig.CurrentLimits.SupplyTimeThreshold = 0.01;

        TurretFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        TurretFXConfig.CurrentLimits.StatorCurrentLimit = 25;

        /* PID Config */
        TurretFXConfig.Slot0.kP = 0.2;
        TurretFXConfig.Slot0.kI = 0;
        TurretFXConfig.Slot0.kD = 0;

        /* Open and Closed Loop Ramping */
        TurretFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        TurretFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        TurretFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
        TurretFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        // Config Motor
        TurretKraken.getConfigurator().apply(TurretFXConfig);
        TurretKraken.getConfigurator().setPosition(0.0);
        TurretKrakenFollower.getConfigurator().setPosition(0.0);
	}

	public void setSpeed(double speed) {
        this.TurretKraken.set(speed);
	}

	public double getCurrentDrawLeader() {
		return this.TurretKraken.getSupplyCurrent().getValueAsDouble();
	}

    public double getCurrentDrawFollower() {
		return this.TurretKrakenFollower.getSupplyCurrent().getValueAsDouble();
	}

	public void resetShooterEncoder() {
        try {
			TurretKraken.getConfigurator().setPosition(0.0);
            TurretKrakenFollower.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Shooter.resetShooterEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("Turret Current", this.getCurrentDrawLeader());
        SmartDashboard.putNumber("Turret Follower Current", this.getCurrentDrawFollower());

	}
}