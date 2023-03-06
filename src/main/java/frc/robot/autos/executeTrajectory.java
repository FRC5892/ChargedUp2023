package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Scoring.scoreFull;
import frc.robot.subsystems.Swerve;

public class executeTrajectory extends SequentialCommandGroup {
  public executeTrajectory(Swerve s_Swerve, PathPlannerTrajectory trajectory, Command s, Command r, Command i,
      Command a, Command sf) {
    s_Swerve.getField().getObject("Field").setTrajectory(trajectory);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("scoreFull", sf);
    eventMap.put("score", s);
    eventMap.put("retract", r);
    eventMap.put("intake", i);
    eventMap.put("active", a);

    SwerveAutoBuilder swerveControllerCommand = new SwerveAutoBuilder(
        s_Swerve::getPose,
        s_Swerve::resetOdometry,
        Constants.Swerve.swerveKinematics,
        new PIDConstants(Constants.AutoConstants.kPXController, 0, 0),
        new PIDConstants(Constants.AutoConstants.kPYController, 0, 0),
        s_Swerve::setModuleStates,
        eventMap,
        true,
        s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose())),
        swerveControllerCommand.fullAuto(trajectory));
  }
}
