package org.ghrobotics.robot.auto

import kotlinx.coroutines.GlobalScope
import org.ghrobotics.lib.commands.S3ND
import org.ghrobotics.lib.commands.StateCommandGroupBuilder
import org.ghrobotics.lib.commands.stateCommandGroup
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
/* ktlint-disable no-wildcard-imports */
import org.ghrobotics.lib.utils.*
import org.ghrobotics.lib.wrappers.FalconRobotBase
import org.ghrobotics.robot.Network
import org.ghrobotics.robot.auto.routines.AutoRoutine
import org.ghrobotics.robot.auto.routines.RoutineCharacterization
import org.ghrobotics.robot.subsytems.drive.DriveSubsystem

@Suppress("LocalVariableName")
/**
 * Manages autonomous.
 */
object Autonomous {

    // Autonomous config
    object Config {
        val autoMode = { Network.autoModeChooser.selected }
        val startingPosition = { Network.startingPositionChooser.selected }
    }

    private var configValid = Source(true)
    private val isReady = { FalconRobotBase.INSTANCE.run { isAutonomous && isEnabled } } and configValid

    // Autonomous Master Group
    private val JUST = stateCommandGroup(Config.autoMode) {
        state(AutoMode.REAL) {
            stateCommandGroup(Config.startingPosition) {
                TODO("Write autos")
            }
        }
        state(AutoMode.CHARACTERIZE, RoutineCharacterization(Config.startingPosition))
    }

    init {
        val IT = ""

        val startingPositionMonitor = Config.startingPosition.monitor
        val isReadyMonitor = isReady.monitor

        GlobalScope.launchFrequency {
            startingPositionMonitor.onChange { DriveSubsystem.localization.reset(it.pose) }
            isReadyMonitor.onChangeToTrue { JUST S3ND IT }
        }

        FalconRobotBase.INSTANCE.onLeave(FalconRobotBase.Mode.AUTONOMOUS) { JUST.stop() }
    }

    private fun <T> StateCommandGroupBuilder<T>.state(state: T, routine: AutoRoutine) = state(state, routine.create())
}

enum class StartingPositions(val pose: Pose2d) { LEFT(Pose2d()), CENTER(Pose2d()), RIGHT(Pose2d()) /* TODO Find Actual Value */ }
enum class AutoMode { CHARACTERIZE, REAL }
