package services.parrot;

import org.ros.node.ConnectedNode;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.LandService;
import services.ServiceFactory;
import services.TakeOffService;
import std_msgs.Empty;

/**
 * @author Hoang Tung Dinh
 */
abstract class ParrotServiceFactory implements ServiceFactory {

    private static final Logger logger = LoggerFactory.getLogger(ParrotServiceFactory.class);

    private final ConnectedNode connectedNode;
    private final String droneName;

    ParrotServiceFactory(ConnectedNode connectedNode, String droneName) {
        this.connectedNode = connectedNode;
        this.droneName = droneName;
    }

    @Override
    public TakeOffService createTakeOffService() {
        final String topicName = "/" + droneName + "/takeoff";
        final TakeOffService takeOffService = ParrotTakeOffService.create(
                connectedNode.<Empty>newPublisher(topicName, Empty._TYPE));
        logger.info("Take off service connected to {}", topicName);
        return takeOffService;
    }

    @Override
    public LandService createLandService() {
        final String topicName = "/" + droneName + "/land";
        final LandService landService = ParrotLandService.create(
                connectedNode.<Empty>newPublisher(topicName, Empty._TYPE));
        logger.info("Land service connected to {}", topicName);
        return landService;
    }

    ConnectedNode getConnectedNode() {
        return connectedNode;
    }

    String getDroneName() {
        return droneName;
    }
}
