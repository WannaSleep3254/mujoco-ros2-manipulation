"""Optional one-argument Humble image subscriptions; no command publishers/clients."""
import hashlib
from numbers import Integral


def _endpoint_id(info):
    raw = info.endpoint_gid
    if (len(raw) not in (16,24) or any(not isinstance(value,Integral) or isinstance(value,bool)
                                     or not 0 <= value <= 255 for value in raw)):
        raise ValueError("Invalid publisher discovery endpoint")
    return hashlib.sha256(bytes(int(value) for value in raw)).hexdigest()


class SensorSubscriptions:
    def __init__(self,node,source,image_type,clock_type,qos):
        self.node,self.source,self.image_type,self.qos=node,source,image_type,qos
        self.subscription=None
        self.subscription_generation=None
        self.closed=False
        self.faulted=False
        self.clean=True
        self.clock_subscription=node.create_subscription(clock_type,"/clock",self._clock,qos)
        source.start()

    def _clock(self,message):
        try:
            seconds,nanos=message.clock.sec,message.clock.nanosec
            self.source.observe_clock_stamp(seconds,nanos)
        except (AttributeError,TypeError,OverflowError,ValueError):
            self.source.observe_clock_stamp(None,None)

    def discover(self):
        if self.closed or self.faulted:
            return
        try:
            endpoints=self.node.get_publishers_info_by_topic(self.source.topic)
            if len(endpoints)>8:
                generation=self.source.discovery_ambiguous()
                ids=[]
            else:
                ids=[_endpoint_id(info) for info in endpoints]
                generation=self.source.set_publishers(ids)
        except Exception:
            self.source.discovery_failed()
            generation=self.source.generation
            ids=[]
        if self.subscription is not None and (generation!=self.subscription_generation or len(ids)!=1):
            try:
                destroyed=self.node.destroy_subscription(self.subscription)
                if destroyed is False:
                    raise RuntimeError("Image subscription cleanup failed")
            except Exception:
                self.faulted=True
                self.source.discovery_failed()
                return
            self.subscription=None
            self.subscription_generation=None
        if len(ids)==1 and self.subscription is None:
            self.subscription_generation=generation
            def receive(message, bound_generation=generation):
                self.source.receive_message(message,generation=bound_generation)
            try:
                self.subscription=self.node.create_subscription(self.image_type,self.source.topic,receive,self.qos)
            except Exception:
                self.source.discovery_failed()
                self.subscription_generation=None

    def close(self):
        if self.closed:
            return self.source.close() and self.clean
        self.closed=True
        clean=True
        for subscription in (self.subscription,self.clock_subscription):
            if subscription is not None:
                try:
                    clean=self.node.destroy_subscription(subscription) is not False and clean
                except Exception:
                    clean=False
        self.subscription=self.clock_subscription=None
        self.clean=clean
        return self.source.close() and self.clean


def attach(node,source):
    """Root bridge calls ``discover`` on its timer and ``close`` before node teardown."""
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    return SensorSubscriptions(node,source,Image,Clock,qos)
