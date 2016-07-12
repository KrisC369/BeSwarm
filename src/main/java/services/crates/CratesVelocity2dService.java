package services.crates;

import commands.VelocityServiceVisitor;
import hal_quadrotor.VelocityHeightRequest;
import hal_quadrotor.VelocityHeightResponse;
import org.ros.node.service.ServiceClient;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.Velocity2dService;

/**
 * @author Hoang Tung Dinh
 */
final class CratesVelocity2dService implements Velocity2dService {

    private static final Logger logger = LoggerFactory.getLogger(CratesVelocity2dService.class);

    private final ServiceClient<VelocityHeightRequest, VelocityHeightResponse> srvVelocity;

    private CratesVelocity2dService(ServiceClient<VelocityHeightRequest, VelocityHeightResponse> srvVelocity) {
        this.srvVelocity = srvVelocity;
    }

    public static CratesVelocity2dService create(
            ServiceClient<VelocityHeightRequest, VelocityHeightResponse> srvVelocity) {
        return new CratesVelocity2dService(srvVelocity);
    }

    @Override
    public void sendVelocityHeightMessage(double inertialFrameVelocityX, double inertialFrameVelocityY,
            double linearPositionZ, double angularPositionZ) {
        final VelocityHeightRequest velocityHeightRequest = srvVelocity.newMessage();
        velocityHeightRequest.setDx(inertialFrameVelocityX);
        velocityHeightRequest.setDy(inertialFrameVelocityY);
        velocityHeightRequest.setZ(linearPositionZ);
        velocityHeightRequest.setYaw(angularPositionZ);
        logger.debug("Sending 2d velocity: [velX = {}, velY = {}, posZ = {}, posYaw = {}]", inertialFrameVelocityX,
                inertialFrameVelocityY, linearPositionZ, angularPositionZ);
        CratesUtilities.sendRequest(srvVelocity, velocityHeightRequest);
    }

    @Override
    public void accept(VelocityServiceVisitor visitor) {
        visitor.visit(this);
    }
}
