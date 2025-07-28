from anytree import Node, RenderTree, LevelOrderIter, AsciiStyle, find


class MessageParser:

    def __init__(self):
        self.tree = None
        self.addNode("<vehicleId>",1)
        self.addNode("<vehicleId>/<deviceId>",2)
        self.addNode("<vehicleId>/<deviceId>/version",3)
        self.addNode("<vehicleId>/<deviceId>/data",3)
        self.addNode("<vehicleId>/<deviceId>/data/primary",4)
        self.addNode("<vehicleId>/<deviceId>/data/secondary",4)
        self.addNode("<vehicleId>/<deviceId>/data/bms",4)
        self.addNode("<vehicleId>/<deviceId>/data/inverters",4)
        self.addNode("<vehicleId>/<deviceId>/data/simulator",4)
        self.addNode("<vehicleId>/<deviceId>/data/gps",4)
        self.addNode("<vehicleId>/<deviceId>/data/brusa",4)
        self.addNode("<vehicleId>/<deviceId>/data/temporary",4)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger",4)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can",5)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/imuAngularRate",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/imuAcceleration",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/vehiclePosition",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/vehicleSpeed",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/frontAngularVelocity",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/rearLeftAngularVelocity",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/rearRightAngularVelocity",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/steerAngle",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS",5)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/hpposllh",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/pvt",6)
        self.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/relposned",6)
        self.addNode("<vehicleId>/<deviceId>/extra_data_to_log",3)
        self.addNode("<vehicleId>/<deviceId>/extra_tlm_data",3)
        self.addNode("<vehicleId>/<deviceId>/extra_tlm_data/vehicleState",4)
        self.addNode("<vehicleId>/<deviceId>/extra_tlm_data/baseline",4)
        self.addNode("<vehicleId>/<deviceId>/extra_tlm_data/gpsMapOrigins",4)
        self.addNode("<vehicleId>/<deviceId>/last_update",3)
        self.addNode("<vehicleId>/<deviceId>/status",3)
        self.addNode("<vehicleId>/<deviceId>/status/info",4)
        self.addNode("<vehicleId>/<deviceId>/status/error",4)
        self.addNode("<vehicleId>/<deviceId>/status/alert",4)
        self.addNode("<vehicleId>/<deviceId>/status/canFrequencies",4)
        self.addNode("<vehicleId>/<deviceId>/status/lapCounterStatus",4)
        self.addNode("<vehicleId>/<deviceId>/status/lapCounterLaps",4)
        self.addNode("<vehicleId>/<deviceId>/as",3)
        self.addNode("<vehicleId>/<deviceId>/as/commands",4)
        self.addNode("<vehicleId>/<deviceId>/as/commands/setValues",5)
        self.addNode("<vehicleId>/<deviceId>/as/commands/setStatus",5)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/request",4)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/response",4)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>",4)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/begin",5)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/end",5)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/chunk",5)
        self.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/chunk_ack",5)
        self.addNode("<vehicleId>/<deviceId>/info",3)
        self.addNode("<vehicleId>/<deviceId>/info/version",4)
        self.addNode("<vehicleId>/<deviceId>/info/user",4)
        self.addNode("<vehicleId>/<deviceId>/info/telemetryReplay",4)
        self.addNode("<vehicleId>/<deviceId>/info/telemetryReplay/ready",5)
        self.addNode("<vehicleId>/<deviceId>/info/telemetryReplay/start",5)
        self.addNode("<vehicleId>/<deviceId>/info/session",4)
        self.addNode("<vehicleId>/<deviceId>/info/session/started",5)
        self.addNode("<vehicleId>/<deviceId>/info/session/stopped",5)
        self.addNode("<vehicleId>/<deviceId>/info/session/logging",5)
        self.addNode("<vehicleId>/<deviceId>/action",3)
        self.addNode("<vehicleId>/<deviceId>/action/telemetryConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/sessionConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/sessionConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/sessionConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/sessionConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/carConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/carConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/carConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/carConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/baselineConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/baselineConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/baselineConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/baselineConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/handcartSettings",4)
        self.addNode("<vehicleId>/<deviceId>/action/handcartSettings/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/handcartSettings/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/handcartSettings/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig",4)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack",4)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/set",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/get",5)
        self.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/content",5)
        self.addNode("<vehicleId>/<deviceId>/action/kill",4)
        self.addNode("<vehicleId>/<deviceId>/action/start",4)
        self.addNode("<vehicleId>/<deviceId>/action/reset",4)
        self.addNode("<vehicleId>/<deviceId>/action/stop",4)
        self.addNode("<vehicleId>/<deviceId>/action/startBaseline",4)
        self.addNode("<vehicleId>/<deviceId>/action/stopBaseline",4)
        self.addNode("<vehicleId>/<deviceId>/action/precharge",4)
        self.addNode("<vehicleId>/<deviceId>/action/balance",4)
        self.addNode("<vehicleId>/<deviceId>/action/stopBalance",4)
        self.addNode("<vehicleId>/<deviceId>/action/charge",4)
        self.addNode("<vehicleId>/<deviceId>/action/stopCharge",4)
        self.addNode("<vehicleId>/<deviceId>/action/raw",4)
        self.addNode("<vehicleId>/<deviceId>/action/resetLapcounter",4)
        self.addNode("<vehicleId>/<deviceId>/action/setLapcounterStatus",4)
        self.addNode("<vehicleId>/simulator",2)
        self.addNode("<vehicleId>/simulator/inputs",3)
        self.addNode("<vehicleId>/simulator/outputs",3)
        self.addNode("<vehicleId>/simulator/initialState",3)

    def addNode(self, topic, layer):
        if topic != "":
            topic_split = topic.split("/")
            if self.tree != None:
                _parent = find(
                    self.tree,
                    lambda node: node.name == topic_split[len(topic_split) - 2]
                    and node.layer == layer - 1,
                )
                Node(
                    topic_split[len(topic_split) - 1],
                    parent=_parent,
                    arg=None,
                    layer=layer,
                    function=None,
                )
            else:
                self.tree = Node(
                    topic_split[len(topic_split) - 1],
                    arg=None,
                    layer=layer,
                    function=None,
                )

    def findNode(self, topic):
        if topic != "":
            topic_split = topic.split("/")
        topic_split[0] = "<vehicleId>"
        topic_split[1] = "<deviceId>"
        _topic_split = "/".join(topic_split)
        topic_split = _topic_split.split("/")
        _node = None
        match = [False] * len(topic_split)
        for i in range(len(topic_split)):
            for node in LevelOrderIter(self.tree, maxlevel=i + 1):
                if node.name == topic_split[i]:
                    _node = node
                    match[i] = True
        return _node

    def setMessageParse(self, topic, parse_function, argument):
        topic_split = topic.split("/")
        topic_split[0] = "<vehicleId>"
        topic_split[1] = "<deviceId>"
        _topic = "/".join(topic_split)
        _node = self.findNode(_topic)
        if _node != None:
            _node.function = parse_function
            _node.arg = argument

    def parseMessage(self, topic, payload):
        topic_split = topic.split("/")
        topic_split[0] = "<vehicleId>"
        topic_split[1] = "<deviceId>"
        _topic = "/".join(topic_split)
        _node = self.findNode(_topic)
        if _node != None and _node.function != None and _node.arg != None:
            _node.function(payload, _node.arg)