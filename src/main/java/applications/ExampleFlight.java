package applications;

import com.google.common.base.Optional;
import com.google.common.collect.ImmutableList;
import commands.Command;
import commands.FollowTrajectory;
import commands.Hover;
import commands.Land;
import commands.Takeoff;
import control.Trajectory4d;
import control.localization.StateEstimator;
import keyboard.Key;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.LandService;
import services.ServiceFactory;
import services.TakeOffService;
import services.VelocityService;
import services.ros_subscribers.KeyboardSubscriber;
import taskexecutor.Task;
import taskexecutor.TaskExecutor;
import taskexecutor.TaskExecutorService;
import taskexecutor.TaskType;
import taskexecutor.interruptors.KeyboardEmergency;

import java.util.ArrayList;
import java.util.Collection;

/**
 * This class illustrates an example flight. The drone will take off, hover in 5 second, follow a trajectory and then
 * land.
 *
 * @author Hoang Tung Dinh
 */
public final class ExampleFlight {

    private static final Logger logger = LoggerFactory.getLogger(ExampleFlight.class);

    private final TakeOffService takeOffService;
    private final LandService landService;
    private final VelocityService velocityService;
    private final StateEstimator stateEstimator;
    private final Trajectory4d trajectory4d;
    private final ConnectedNode connectedNode;

    private ExampleFlight(ServiceFactory serviceFactory, StateEstimator stateEstimator, Trajectory4d trajectory4d,
            ConnectedNode connectedNode) {
        final Optional<TakeOffService> takeOffServiceOptional = serviceFactory.createTakeOffService();
        final Optional<LandService> landServiceOptional = serviceFactory.createLandService();
        final Optional<VelocityService> velocityServiceOptional = serviceFactory.createVelocityService();

        if (!(takeOffServiceOptional.isPresent() && landServiceOptional.isPresent() && velocityServiceOptional
                .isPresent())) {
            logger.info("Some services are not available. The program will quit automatically.");
            logger.info(takeOffServiceOptional.toString());
            logger.info(landServiceOptional.toString());
            logger.info(velocityServiceOptional.toString());
        }

        this.takeOffService = takeOffServiceOptional.get();
        this.landService = landServiceOptional.get();
        this.velocityService = velocityServiceOptional.get();
        this.stateEstimator = stateEstimator;
        this.trajectory4d = trajectory4d;
        this.connectedNode = connectedNode;
    }

    public static ExampleFlight create(ServiceFactory serviceFactory, StateEstimator stateEstimator,
            Trajectory4d trajectory4d, ConnectedNode connectedNode) {
        return new ExampleFlight(serviceFactory, stateEstimator, trajectory4d, connectedNode);
    }

    public void fly() {
        // task to execute in case of emergency
        final Task emergencyTask = createEmergencyTask();
        final KeyboardEmergency keyboardEmergencyNotifier = createKeyboardEmergencyNotifier(emergencyTask);

        final TaskExecutor taskExecutor = TaskExecutorService.create();
        keyboardEmergencyNotifier.registerTaskExecutor(taskExecutor);

        // normal fly task
        final Task flyTask = createFlyTask();
        taskExecutor.submitTask(flyTask);
    }

    private Task createFlyTask() {
        final Collection<Command> commands = new ArrayList<>();

        final Command takeOff = Takeoff.create(takeOffService);
        commands.add(takeOff);

        final Command hoverFiveSecond = Hover.create(velocityService, stateEstimator, 5);
        commands.add(hoverFiveSecond);

        final Command followTrajectory = FollowTrajectory.builder()
                .stateEstimator(stateEstimator)
                .velocityService(velocityService)
                .trajectory4d(trajectory4d)
                .durationInSeconds(60)
                .build();
        commands.add(followTrajectory);

        return Task.create(ImmutableList.copyOf(commands), TaskType.NORMAL_TASK);
    }

    private Task createEmergencyTask() {
        final Command land = Land.create(landService);
        return Task.create(ImmutableList.of(land), TaskType.FIRST_ORDER_EMERGENCY);
    }

    private KeyboardEmergency createKeyboardEmergencyNotifier(Task emergencyTask) {
        final KeyboardSubscriber keyboardSubscriber = KeyboardSubscriber.createKeyboardSubscriber(
                connectedNode.<Key>newSubscriber("/keyboard/keydown", Key._TYPE));
        final KeyboardEmergency keyboardEmergency = KeyboardEmergency.create(emergencyTask);
        keyboardSubscriber.registerObserver(keyboardEmergency);
        return keyboardEmergency;
    }
}
