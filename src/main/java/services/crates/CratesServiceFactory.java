package services.crates;

import hal_quadrotor.Land;
import hal_quadrotor.LandRequest;
import hal_quadrotor.LandResponse;
import hal_quadrotor.State;
import hal_quadrotor.Takeoff;
import hal_quadrotor.TakeoffRequest;
import hal_quadrotor.TakeoffResponse;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.FlyingStateService;
import services.LandService;
import services.CommonServiceFactory;
import services.TakeOffService;
import services.ros_subscribers.MessagesSubscriberService;

/**
 * @author Hoang Tung Dinh
 */
public final class CratesServiceFactory implements CommonServiceFactory {

    private static final Logger logger = LoggerFactory.getLogger(CratesServiceFactory.class);

    private final String droneName;
    private final String modelName;
    private final ConnectedNode connectedNode;
    private final String namePrefix;

    private CratesServiceFactory(String droneName, String modelName, ConnectedNode connectedNode) {
        this.droneName = droneName;
        this.modelName = modelName;
        this.connectedNode = connectedNode;
        this.namePrefix = "/hal/quadrotor/" + modelName + "/" + droneName + "/";
    }

    public static CratesServiceFactory create(String droneName, String modelName, ConnectedNode connectedNode) {
        return new CratesServiceFactory(droneName, modelName, connectedNode);
    }

    @Override
    public TakeOffService createTakeOffService() {
        try {
            return CratesTakeOffService.create(
                    connectedNode.<TakeoffRequest, TakeoffResponse>newServiceClient(namePrefix + "controller/Takeoff",
                            Takeoff._TYPE));
        } catch (ServiceNotFoundException e) {
            logger.info("Take off service not found. Drone: {}. Model: {}. Exception: {}", droneName, modelName, e);
            throw new RuntimeException(
                    String.format("Take off service not found. Drone: %s. Model: %s", droneName, modelName));
        }
    }

    @Override
    public LandService createLandService() {
        try {
            return CratesLandService.create(
                    connectedNode.<LandRequest, LandResponse>newServiceClient(namePrefix + "controller/Land",
                            Land._TYPE));
        } catch (ServiceNotFoundException e) {
            throw new RuntimeException(
                    String.format("Land service not found. Drone: %s. Model: %s", droneName, modelName));
        }
    }

    @Override
    public FlyingStateService createFlyingStateService() {
        final String topicName = namePrefix + "Truth";
        final MessagesSubscriberService<State> flyingStateSubscriber = MessagesSubscriberService.create(
                connectedNode.<State>newSubscriber(topicName, State._TYPE));
        return CratesFlyingStateService.create(flyingStateSubscriber);
    }
}
