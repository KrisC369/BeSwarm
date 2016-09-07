package applications.parrot.tumsim;

import applications.ExampleFlight;
import applications.RosParameters;
import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;
import commands.Command;
import commands.WaitForLocalizationDecorator;
import commands.tumsimcommands.TumSimFollowTrajectory;
import commands.tumsimcommands.TumSimHover;
import commands.tumsimcommands.TumSimLand;
import commands.tumsimcommands.TumSimTakeoff;
import control.FiniteTrajectory4d;
import control.PidParameters;
import control.dto.DroneStateStamped;
import control.localization.GazeboModelStateEstimator;
import control.localization.StateEstimator;
import gazebo_msgs.ModelStates;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.FlyingStateService;
import services.LandService;
import services.ResetService;
import services.TakeOffService;
import services.Velocity4dService;
import services.parrot.ParrotServiceFactory;
import services.parrot.TumSimServiceFactory;
import services.rossubscribers.MessagesSubscriberService;
import taskexecutor.Task;
import taskexecutor.TaskType;
import time.RosTime;

import java.util.ArrayList;
import java.util.Collection;
import java.util.concurrent.TimeUnit;

/** @author Hoang Tung Dinh */
final class TumExampleFlightFacade {
  private static final Logger logger = LoggerFactory.getLogger(TumExampleFlightFacade.class);
  private static final String MODEL_NAME = "quadrotor";
  private final ExampleFlight exampleFlight;
  private final StateEstimator stateEstimator;

  private TumExampleFlightFacade(FiniteTrajectory4d trajectory4d, ConnectedNode connectedNode) {
    final PidParameters pidLinearX =
        RosParameters.createPidParameters(
            connectedNode,
            "beswarm/pid_linear_x_kp",
            "beswarm/pid_linear_x_kd",
            "beswarm/pid_linear_x_ki",
            "beswarm/pid_lag_time_in_seconds");
    final PidParameters pidLinearY =
        RosParameters.createPidParameters(
            connectedNode,
            "beswarm/pid_linear_y_kp",
            "beswarm/pid_linear_y_kd",
            "beswarm/pid_linear_y_ki",
            "beswarm/pid_lag_time_in_seconds");
    final PidParameters pidLinearZ =
        RosParameters.createPidParameters(
            connectedNode,
            "beswarm/pid_linear_z_kp",
            "beswarm/pid_linear_z_kd",
            "beswarm/pid_linear_z_ki",
            "beswarm/pid_lag_time_in_seconds");
    final PidParameters pidAngularZ =
        RosParameters.createPidParameters(
            connectedNode,
            "beswarm/pid_angular_z_kp",
            "beswarm/pid_angular_z_kd",
            "beswarm/pid_angular_z_ki",
            "beswarm/pid_lag_time_in_seconds");

    final ParrotServiceFactory parrotServiceFactory = TumSimServiceFactory.create(connectedNode);
    stateEstimator = getStateEstimator(connectedNode);
    final LandService landService = parrotServiceFactory.createLandService();
    final FlyingStateService flyingStateService = parrotServiceFactory.createFlyingStateService();
    final TakeOffService takeOffService = parrotServiceFactory.createTakeOffService();
    final ResetService resetService = parrotServiceFactory.createResetService();
    final Velocity4dService velocity4dService = parrotServiceFactory.createVelocity4dService();

    final Collection<Command> commands = new ArrayList<>();

    final Command takeOff = TumSimTakeoff.create(takeOffService, flyingStateService, resetService);
    commands.add(takeOff);

    final Command hoverFiveSecond =
        TumSimHover.create(5, RosTime.create(connectedNode), velocity4dService, stateEstimator);
    commands.add(hoverFiveSecond);

    final Command followTrajectory =
        TumSimFollowTrajectory.builder()
            .withVelocity4dService(velocity4dService)
            .withStateEstimator(stateEstimator)
            .withTimeProvider(RosTime.create(connectedNode))
            .withTrajectory4d(trajectory4d)
            .withDurationInSeconds(trajectory4d.getTrajectoryDuration())
            .withPidLinearXParameters(pidLinearX)
            .withPidLinearYParameters(pidLinearY)
            .withPidLinearZParameters(pidLinearZ)
            .withPidAngularZParameters(pidAngularZ)
            .withControlRateInSeconds(0.01)
            .build();

    final Command waitForLocalizationThenFollowTrajectory =
        WaitForLocalizationDecorator.create(stateEstimator, followTrajectory);

    commands.add(waitForLocalizationThenFollowTrajectory);

    final Command land = TumSimLand.create(landService, flyingStateService);
    commands.add(land);

    final Task flyTask = Task.create(ImmutableList.copyOf(commands), TaskType.NORMAL_TASK);
    final Task emergencyTask = createEmergencyTask(landService, flyingStateService);

    exampleFlight = ExampleFlight.create(connectedNode, flyTask, emergencyTask);
  }

  private static Task createEmergencyTask(
      LandService landService, FlyingStateService flyingStateService) {
    final Command land = TumSimLand.create(landService, flyingStateService);
    return Task.create(ImmutableList.of(land), TaskType.FIRST_ORDER_EMERGENCY);
  }

  /**
   * Creates a facade to run the drone in the Tum simulator. The drone will take off, follow a
   * provided trajectory and then land.
   *
   * @param trajectory4d the trajectory which the drone will follow
   * @param connectedNode the connected node
   * @return a facade for flying with the drone in the Tum simulator
   */
  public static TumExampleFlightFacade create(
      FiniteTrajectory4d trajectory4d, ConnectedNode connectedNode) {
    return new TumExampleFlightFacade(trajectory4d, connectedNode);
  }

  private static StateEstimator getStateEstimator(ConnectedNode connectedNode) {
    final MessagesSubscriberService<ModelStates> modelStateSubscriber =
        MessagesSubscriberService.create(
            connectedNode.<ModelStates>newSubscriber("/gazebo/model_states", ModelStates._TYPE),
            RosTime.create(connectedNode));
    return GazeboModelStateEstimator.create(
        modelStateSubscriber, MODEL_NAME, RosTime.create(connectedNode));
  }

  /** Starts flying. */
  void fly() {
    waitUntilRecevingDroneState();
    exampleFlight.fly();
  }

  private void waitUntilRecevingDroneState() {
    Optional<DroneStateStamped> droneState = stateEstimator.getCurrentState();

    // wait until we receive at lease a state of the drone before flying. It is to guarantee that
    // the drone model is properly initialized before running this code.
    while (!droneState.isPresent()) {
      try {
        TimeUnit.MILLISECONDS.sleep(100);
      } catch (InterruptedException e) {
        logger.info("Sleep while waiting for drone state is interrupted.", e);
        Thread.currentThread().interrupt();
      }

      droneState = stateEstimator.getCurrentState();
    }
  }
}
