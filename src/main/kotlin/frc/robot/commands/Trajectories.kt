package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.auto.PIDConstants
import com.pathplanner.lib.auto.SwerveAutoBuilder
import com.pathplanner.lib.commands.FollowPathWithEvents
import com.pathplanner.lib.commands.PPSwerveControllerCommand
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.constants.DrivetrainConstants
import frc.robot.constants.TrajectoryConstants
import frc.robot.subsystems.PickAndPlaceSubsystem
import frc.robot.subsystems.SwerveSubsystem


class Trajectories(val pnp: PickAndPlaceSubsystem, val swerveSubsystem: SwerveSubsystem) {
    fun base(pathName: String, eventMap: HashMap<String, Command>): Command {
        val examplePath = PathPlanner.loadPath(pathName, PathConstraints(2.0, 1.5))

        val command = FollowPathWithEvents(
            TrajectoryDrivePathPlanner(swerveSubsystem, examplePath, true),
            examplePath.markers,
            eventMap
        )
        return command
    }


    fun BlueTop1(): Command {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return base("Blue Top 1", eventMap)
    }
    fun BlueBottom1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Blue Bottom 1", eventMap)}
    }
    fun RedTop1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Red Top 1", eventMap)}
    }
    fun RedBottom1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Red Bottom 1", eventMap)}
    }
    fun BlueTop2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["HighPlace"] = AutoPlaceHigh(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Blue Top 2", eventMap)}
    }
    fun BlueBottom2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["HighPlace"] = AutoPlaceHigh(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Blue Bottom 2", eventMap)}
    }
    fun RedTop2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["HighPlace"] = AutoPlaceHigh(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Red Top 2", eventMap)}
    }
    fun RedBottom2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["HighPlace"] = AutoPlaceHigh(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Base"] = Base(pnp)
        return {base("Red Bottom 2", eventMap)}
    }
    fun BlueCenter1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Center 1 Balance", eventMap)}
    }
    fun RedCenter1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Red Center 1 Balance", eventMap)}
    }
    fun BlueTop1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Top 1 Balance", eventMap)}
    }
    fun BlueBottom1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Bottom 1 Balance", eventMap)}
    }
    fun RedTop1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Red Top 1 Balance", eventMap)}
    }
    fun RedBottom1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Red Bottom 1 Balance", eventMap)}
    }
    fun BlueTop1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Top 1 Get Balance", eventMap)}
    }
    fun BlueBottom1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = AutoPlaceMid(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Bottom 1 Get Balance", eventMap)}
    }
    fun RedTop1GetBalance(): Command {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlaceCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return base("Red Top 1 Get Balance", eventMap)
    }
    fun RedBottom1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlaceCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = AutoPick(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)
        return {base("Blue Top 1", eventMap)}
    }
    fun TestPath(): Command {
        val examplePath = PathPlanner.loadPath("Red Top 1 Get Balance", PathConstraints(1.00, 0.50))

        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlaceCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Balance(swerveSubsystem)

        val command = FollowPathWithEvents(
            TrajectoryDrivePathPlanner(swerveSubsystem, examplePath, true),
            examplePath.markers,
            eventMap
        )
        return command
    }
}

fun TestPathAutoBuilder(swerveSubsystem: SwerveSubsystem, pnp: PickAndPlaceSubsystem): Command {
    var thetaController = PIDController(
        TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController
    )
    thetaController.enableContinuousInput(-Math.PI, Math.PI)

    val path = PathPlanner.loadPath("Red Top 1 Get Balance", PathConstraints(1.0, 0.5))

    val eventMap = HashMap<String, Command>()
    eventMap["MidPlace"] = MidPlaceCube(pnp)
    eventMap["Base"] = Base(pnp)
    eventMap["PickUpCube"] = LowPickCube(pnp)
    eventMap["Base"] = Base(pnp)
    eventMap["Balance"] = Balance(swerveSubsystem)

    val autoBuilder = SwerveAutoBuilder(
        swerveSubsystem::pose,
        swerveSubsystem::resetOdometry,
        DrivetrainConstants.driveKinematics,
        PIDConstants(TrajectoryConstants.kPXController, 0.0, 0.0),
        PIDConstants(TrajectoryConstants.kPThetaController, 0.0, TrajectoryConstants.kDThetaController),
        swerveSubsystem::setModuleStates,
        eventMap,
        false,
        swerveSubsystem
    )

    val fullAuto: Command = autoBuilder.fullAuto(path)
    return fullAuto
}