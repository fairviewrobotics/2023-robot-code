package frc.robot.commands

import com.pathplanner.lib.PathConstraints
import com.pathplanner.lib.PathPlanner
import com.pathplanner.lib.PathPlannerTrajectory
import com.pathplanner.lib.commands.FollowPathWithEvents
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import frc.robot.commands.Base
import frc.robot.commands.TrajectoryDrivePathPlanner
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.PickAndPlaceSubsystem
import frc.robot.subsystems.SwerveSubsystem
import java.lang.ProcessBuilder.Redirect

class Trajectories {
    val pnp = PickAndPlaceSubsystem(ArmConstants.elevatorMotorId, ArmConstants.elbowMotorId, 11, 12, 13, 0, 1)
    val swerveSubsystem = SwerveSubsystem()
    fun base(pathName: String, eventMap: HashMap<String, Command>) {
        val lambda: () -> Unit = {
            val examplePath = PathPlanner.loadPath(pathName, PathConstraints(4.0, 3.0))

            val command = FollowPathWithEvents(
                RunCommand({
                    TrajectoryDrivePathPlanner(swerveSubsystem, examplePath, false)
                }),
                examplePath.markers,
                eventMap
            )
        }
        lambda
    }

    fun BlueTop1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Top 1", eventMap)}
    }
    fun BlueBottom1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Bottom 1", eventMap)}
    }
    fun RedTop1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Top 1", eventMap)}
    }
    fun RedBottom1(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Bottom 1", eventMap)}
    }
    fun BlueTop2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Top 2", eventMap)}
    }
    fun BlueBottom2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Bottom 2", eventMap)}
    }
    fun RedTop2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Top 2", eventMap)}
    }
    fun RedBottom2(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Bottom 2", eventMap)}
    }
    fun BlueCenter1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Center 1 Balance", eventMap)}
    }
    fun RedCenter1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Center 1 Balance", eventMap)}
    }
    fun BlueTop1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Top 1 Balance", eventMap)}
    }
    fun BlueBottom1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Blue Bottom 1 Balance", eventMap)}
    }
    fun RedTop1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Top 1 Balance", eventMap)}
    }
    fun RedBottom1Balance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["Base"] = Base(pnp)
        eventMap["MidPlace"] = MidPlace(pnp)
        return {base("Red Bottom 1 Balance", eventMap)}
    }
    fun BlueTop1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlace(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Base(pnp)//TODO:Change!
        return {base("Blue Top 1 Get Balance", eventMap)}
    }
    fun BlueBottom1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlace(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Base(pnp)//TODO:Change!
        return {base("Blue Bottom 1 Get Balance", eventMap)}
    }
    fun RedTop1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlace(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Base(pnp)//TODO:Change!
        return {base("Red Top 1 Get Balance", eventMap)}
    }
    fun RedBottom1GetBalance(): () -> Unit {
        val eventMap = HashMap<String, Command>()
        eventMap["MidPlace"] = MidPlace(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["PickUpCube"] = LowPickCube(pnp)
        eventMap["Base"] = Base(pnp)
        eventMap["Balance"] = Base(pnp)//TODO:Change!
        return {base("Blue Top 1", eventMap)}
    }





}