"""Robot battery telemetry normalization; no host/laptop battery fallback."""
import math
import json
import time


PHYSICAL_STOP_LOW_VOLTAGE = 8.0
PHYSICAL_STOP_RECOVERED_VOLTAGE = 11.0


def number(value, low=None, high=None):
    try: result = float(value)
    except (ValueError, TypeError): return None
    if not math.isfinite(result) or (low is not None and result<low) or (high is not None and result>high):
        return None
    return result


def normalize(message, kind):
    if kind == 'SolixStatus':
        return normalize_solix(message)
    voltage = number(getattr(message, 'voltage', None), low=0)
    if voltage == 0 and kind == 'BatteryState': voltage = None
    common = {'voltage': voltage, 'percentage': None, 'current': None,
              'temperature': None, 'internal_voltage': None, 'fault_flags': None,
              'emergency_stop': None, 'physical_emergency_stop': None,
              'charging': None, 'present': True,
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
                      physical_emergency_stop=(True if voltage is not None and voltage<=PHYSICAL_STOP_LOW_VOLTAGE
                                               else False if voltage is not None and voltage>=PHYSICAL_STOP_RECOVERED_VOLTAGE else None),
                      temperature=number(getattr(message, 'temperature', None)),
                      current=sum(currents) if all(v is not None for v in currents) else None)
    else: raise ValueError('Unsupported robot battery message')
    return common


def normalize_solix(message):
    try:
        data=json.loads(message.data)
    except (ValueError,TypeError,AttributeError):
        data={}
    if not isinstance(data,dict):data={}
    input_power=number(data.get('total_input'),0)
    output_power=number(data.get('total_output'),0)
    flow=data.get('time_remaining_kind')
    if flow not in ('charging','discharging','idle'):
        flow=('charging' if input_power>output_power else 'discharging' if output_power>input_power else 'idle') if input_power is not None and output_power is not None else 'unknown'
    def hours(value):
        value=number(value,0)
        return value if value is not None and 0<value<6553.5 else None
    runtime=hours(data.get('runtime_hours',data.get('time_remaining'))) if flow=='discharging' else None
    charge=hours(data.get('charge_remaining_hours',data.get('time_remaining'))) if flow=='charging' else None
    stamp=number(data.get('stamp'),1,1e11)
    ports=data.get('ports') if isinstance(data.get('ports'),dict) else {}
    states=data.get('output_states') if isinstance(data.get('output_states'),dict) else {}
    return {'source_type':'SolixStatus','present':data.get('status')=='connected',
            'charging':{'charging':'充電中','discharging':'放電中','idle':'入出力均衡'}.get(flow),
            'percentage':number(data.get('battery_level'),0,100),'temperature':number(data.get('temperature')),
            'input_power':input_power,'output_power':output_power,'time_remaining_kind':flow,
            'runtime_hours':runtime,'charge_remaining_hours':charge,
            'charge_complete_at':stamp+charge*3600 if stamp is not None and charge is not None else None,
            'ports':{key:number(ports.get(key),0) for key in ('ac_input','solar_input','ac_output','dc_output','usb_c1','usb_c2','usb_c3','usb_a1')},
            'output_states':{key:states.get(key) if states.get(key) in ('on','off') else None for key in ('ac','dc')},
            'device_mac':data.get('device_mac') if isinstance(data.get('device_mac'),str) else None}


class BatteryStore:
    def __init__(self, stale_after=5.0):
        self.stale_after=stale_after
        self.records={}
        self.sources=[]
        self.errors=[]

    def ingest(self, topic, kind, message, now):
        if topic not in self.records and len(self.records)>=32:
            del self.records[min(self.records,key=lambda key:self.records[key]['received'])]
        data = normalize(message,kind)
        previous = self.records.get(topic)
        voltage = data.get('voltage')
        # A decaying bus voltage cannot reveal the physical button instantly.
        # Hold an inferred stop through the uncertain band; reset on stale data.
        if (kind == 'RoboteqStatus' and voltage is not None
                and PHYSICAL_STOP_LOW_VOLTAGE < voltage < PHYSICAL_STOP_RECOVERED_VOLTAGE
                and previous and now-previous['received'] <= self.stale_after
                and previous.get('physical_emergency_stop') is True):
            data['physical_emergency_stop'] = True
        self.records[topic] = {**data,'topic':topic,'received':now}

    def snapshot(self, now):
        empty={'state':'waiting','voltage':None,'percentage':None,'current':None,
               'temperature':None,'internal_voltage':None,'fault_flags':None,
               'emergency_stop':None,'physical_emergency_stop':None,
               'charging':None,'topic':None,'source_type':None,
               'age_seconds':None,'stale_after_seconds':self.stale_after,
               'sources':self.sources,'errors':self.errors}
        def valid(record):
            fields={'BatteryState':('voltage','percentage'),'SolixStatus':('percentage','input_power','output_power'),
                    'RoboteqStatus':('voltage','current','internal_voltage','fault_flags')}[record['source_type']]
            return record['present'] and any(record.get(key) is not None for key in fields)
        def rank(record):
            usable=valid(record)
            fresh=now-record['received']<=self.stale_after
            return (fresh and usable, fresh, usable, record['source_type']=='BatteryState', record['received'])
        def select(records):
            if not records:return dict(empty)
            record=max(records,key=rank)
            data={k:v for k,v in record.items() if k!='received'}
            age=max(0,now-record['received'])
            state=('stale' if age>self.stale_after else 'absent' if not record['present'] else
                   'live' if valid(record) else 'invalid')
            if state != 'live':data['physical_emergency_stop'] = None
            return {**empty,**data,'state':state,'age_seconds':age}
        records=list(self.records.values())
        return {**select([r for r in records if r['source_type']!='SolixStatus']),
                'battery':select([r for r in records if r['source_type']=='BatteryState']),
                'driver':select([r for r in records if r['source_type']=='RoboteqStatus']),
                'details':select([r for r in records if r['source_type']=='SolixStatus'])}


class BatteryReceiver:
    """Discover standard BatteryState and Sirius RoboteqStatus on any ROS namespace."""
    TYPES={'sensor_msgs/msg/BatteryState':'BatteryState',
           'roboteq_ros2_driver/msg/RoboteqStatus':'RoboteqStatus',
           'std_msgs/msg/String':'SolixStatus'}

    @classmethod
    def message_type(cls,topic,types):
        return next((t for t in types if t in cls.TYPES and
                     (t!='std_msgs/msg/String' or topic.endswith('/sirius/battery_status'))),None)

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
            message_type=self.message_type(topic,types)
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
