"""Robot battery telemetry normalization; no host/laptop battery fallback."""
import math
import time


def number(value, low=None, high=None):
    try: result = float(value)
    except (ValueError, TypeError): return None
    if not math.isfinite(result) or (low is not None and result<low) or (high is not None and result>high):
        return None
    return result


def normalize(message, kind):
    voltage = number(getattr(message, 'voltage', None), low=0)
    if voltage == 0: voltage = None
    common = {'voltage': voltage, 'percentage': None, 'current': None,
              'temperature': None, 'internal_voltage': None, 'fault_flags': None,
              'emergency_stop': None, 'charging': None, 'present': True,
              'source_type': kind}
    if kind == 'BatteryState':
        fraction = number(getattr(message, 'percentage', None), 0, 1)
        common.update(percentage=None if fraction is None else fraction*100,
                      current=number(getattr(message, 'current', None)),
                      temperature=number(getattr(message, 'temperature', None)),
                      present=bool(getattr(message, 'present', True)))
        status=getattr(message, 'power_supply_status', 0)
        common['charging'] = {1:'充電中',2:'放電中',3:'充電停止',4:'満充電'}.get(status)
    elif kind == 'RoboteqStatus':
        # This is the motor driver's supply, which may be cut by the physical e-stop.
        flags = getattr(message, 'fault_flags', None)
        flags = flags if isinstance(flags, int) and flags>=0 else None
        currents = [number(getattr(message, name, None)) for name in
                    ('battery_current_ch1','battery_current_ch2')]
        common.update(internal_voltage=number(getattr(message, 'voltage_internal', None), 0),
                      fault_flags=flags, emergency_stop=None if flags is None else bool(flags & 16),
                      temperature=number(getattr(message, 'temperature', None)),
                      current=sum(currents) if all(v is not None for v in currents) else None)
        if voltage is None:
            for key in ('current','temperature','internal_voltage','fault_flags','emergency_stop'):
                common[key] = None
    else: raise ValueError('Unsupported robot battery message')
    return common


class BatteryStore:
    def __init__(self, stale_after=5.0):
        self.stale_after=stale_after
        self.records={}
        self.sources=[]
        self.errors=[]

    def ingest(self, topic, kind, message, now):
        if topic not in self.records and len(self.records)>=32:
            del self.records[min(self.records,key=lambda key:self.records[key]['received'])]
        self.records[topic] = {**normalize(message,kind),'topic':topic,'received':now}

    def snapshot(self, now):
        empty={'state':'waiting','voltage':None,'percentage':None,'current':None,
               'temperature':None,'internal_voltage':None,'fault_flags':None,
               'emergency_stop':None,'charging':None,'topic':None,'source_type':None,
               'age_seconds':None,'stale_after_seconds':self.stale_after,
               'sources':self.sources,'errors':self.errors}
        if not self.records:return empty
        def rank(record):
            valid=record['present'] and (record['voltage'] is not None or record['percentage'] is not None)
            fresh=now-record['received']<=self.stale_after
            return (fresh and valid, fresh, valid, record['source_type']=='BatteryState', record['received'])
        record=max(self.records.values(),key=rank)
        data={k:v for k,v in record.items() if k!='received'}
        age=max(0,now-record['received'])
        state=('stale' if age>self.stale_after else 'absent' if not record['present'] else
               'live' if record['voltage'] is not None or record['percentage'] is not None else 'invalid')
        return {**empty,**data,'state':state,'age_seconds':age}


class BatteryReceiver:
    """Discover standard BatteryState and Sirius RoboteqStatus on any ROS namespace."""
    TYPES={'sensor_msgs/msg/BatteryState':'BatteryState',
           'roboteq_ros2_driver/msg/RoboteqStatus':'RoboteqStatus'}

    def __init__(self,node):
        self.node=node
        self.store=BatteryStore()
        self.subscriptions={}
        self.timer=node.create_timer(2.0,self.discover)
        self.discover()

    def discover(self):
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        desired={};errors=[]
        for topic,types in self.node.get_topic_names_and_types():
            message_type=next((t for t in types if t in self.TYPES),None)
            if not message_type:continue
            endpoints=self.node.get_publishers_info_by_topic(topic)
            if not endpoints:continue
            durable=all(e.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for e in endpoints)
            key=(topic,message_type,durable);desired[key]=True
            if key in self.subscriptions:continue
            try:
                cls=get_message(message_type)
                def make_callback(topic,kind):
                    def callback(message):self.store.ingest(topic,kind,message,time.monotonic())
                    return callback
                qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
                self.subscriptions[key]=self.node.create_subscription(cls,topic,make_callback(topic,self.TYPES[message_type]),qos)
            except (ImportError,AttributeError,ValueError,RuntimeError) as error:
                errors.append(topic+': メッセージ型を読み込めません ('+str(error)[:100]+')')
        for key in list(self.subscriptions):
            if key not in desired:self.node.destroy_subscription(self.subscriptions.pop(key))
        self.store.sources=sorted({key[0] for key in desired})
        self.store.errors=errors
