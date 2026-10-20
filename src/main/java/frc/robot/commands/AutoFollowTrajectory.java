// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RomiDrivetrain;
import java.util.Optional;

public class AutoFollowTrajectory extends Command {
    private final RomiDrivetrain romiDrivetrain;
    private final Timer timer = new Timer();
    private final Optional<Trajectory<DifferentialSample>> trajectory;

    private static final double kMaxLinearVelocityMps = 0.5;

    public AutoFollowTrajectory(RomiDrivetrain romiDrivetrain) {
        this.romiDrivetrain = romiDrivetrain;
        
        // Loads Path_1.traj from deploy/choreo/
        this.trajectory = Choreo.loadTrajectory("Path_1");
        
        addRequirements(romiDrivetrain);
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .orElse(Alliance.Blue) == Alliance.Red;
    }

    @Override
    public void initialize() {
        timer.restart();
        
        if (trajectory.isPresent()) {
            // Get initial pose accounting for alliance side
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());
            if (initialPose.isPresent()) {
                romiDrivetrain.resetPose(initialPose.get());
            }
        }
    }

    @Override
    public void execute() {
        if (trajectory.isPresent()) {
            // Pass timer.get() AND alliance mirror flag to sampleAt()
            Optional<DifferentialSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());
            
            if (sample.isPresent()) {
                DifferentialSample currentSample = sample.get();
                
                double leftPercent = currentSample.vl / kMaxLinearVelocityMps;
                double rightPercent = currentSample.vr / kMaxLinearVelocityMps;
                
                romiDrivetrain.setLeftMotorOutput(leftPercent);
                romiDrivetrain.setRightMotorOutput(rightPercent);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        romiDrivetrain.setLeftMotorOutput(0.0);
        romiDrivetrain.setRightMotorOutput(0.0);
    }

    @Override
    public boolean isFinished() {
        return trajectory.isEmpty() || timer.hasElapsed(trajectory.get().getTotalTime());
    }
}