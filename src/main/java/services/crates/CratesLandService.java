package services.crates;

import hal_quadrotor.LandRequest;
import hal_quadrotor.LandResponse;
import org.ros.exception.RemoteException;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import services.LandService;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

/**
 * @author Hoang Tung Dinh
 */
public final class CratesLandService implements LandService {
    private static final Logger logger = LoggerFactory.getLogger(CratesLandService.class);
    private final ServiceClient<LandRequest, LandResponse> srvLand;

    private CratesLandService(ServiceClient<LandRequest, LandResponse> srvLand) {
        this.srvLand = srvLand;
    }

    public static CratesLandService create(ServiceClient<LandRequest, LandResponse> srvLand) {
        return new CratesLandService(srvLand);
    }

    @Override
    public void sendLandingMessage() {
        final LandRequest landRequest = srvLand.newMessage();
        final CountDownLatch countDownLatch = new CountDownLatch(1);
        srvLand.call(landRequest, LandServiceResponseListener.create(countDownLatch));
        try {
            countDownLatch.await(5, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            logger.info("Waiting for landing response is interrupted.", e);
        }
    }

    private static final class LandServiceResponseListener implements ServiceResponseListener<LandResponse> {
        private final CountDownLatch countDownLatch;

        private LandServiceResponseListener(CountDownLatch countDownLatch) {
            this.countDownLatch = countDownLatch;
        }

        public static LandServiceResponseListener create(CountDownLatch countDownLatch) {
            return new LandServiceResponseListener(countDownLatch);
        }

        @Override
        public void onSuccess(LandResponse landResponse) {
            logger.info("Successfully landed!!!");
            logger.info(landResponse.getStatus());
            countDownLatch.countDown();
        }

        @Override
        public void onFailure(RemoteException e) {
            logger.info("Cannot send landing message!!!", e);
            countDownLatch.countDown();
        }
    }
}