// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class AutoWallTwoCones extends SequentialCommandGroup implements AutoCommandInterface {
    AutoFollowTrajectory[] m_traj;

    /** Creates a new AutoWallTwoCones */
    public AutoWallTwoCones(DriveTrain driveTrain, Arm arm, Vision vision, Position secondConePos) {

        m_traj = new AutoFollowTrajectory[] { new AutoFollowTrajectory(driveTrain, "bot_grid_s1"),
                new AutoFollowTrajectory(driveTrain, "bot_grid_s2"),
                new AutoFollowTrajectory(driveTrain, "bot_grid_s3") };

        addCommands(m_traj[0], 
            m_traj[1].andThen(new DriveAndMoveArm(arm, driveTrain, vision, secondConePos)),
            m_traj[2]);
                
        // Do NOT require any Subsystems. That is handled by the subcommands.
    }

    @Override
    public Pose2d getInitialPose() {
        return m_traj[0].getInitialPose();
    }

    @Override
    public void plotTrajectory(TrajectoryPlotter plotter) {
        for (int i = 0; i < m_traj.length; i++) {
            m_traj[i].plotTrajectory(plotter, i);
        }
    }
}
