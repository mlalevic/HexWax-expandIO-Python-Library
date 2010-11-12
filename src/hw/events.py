import commands

class Actions:
    Send = "Send"
    Log = "Log"
    Read = "Read"
    Update = "Update"

class Events:
    actions = {}

    @staticmethod
    def register(action):
        Events.actions[action.name] = action

    @staticmethod
    def clear_callbacks():
        Events.actions = {}

    @staticmethod
    def do(event, *args):
        action = Events.actions[event]
        if action is not None:
            action.handle(*args)
        else:
            Events.do(Actions.Log, "No action for event: %s" % event)


class SendHandler:
    def __init__(self, usb):
        self.usb = usb
        self.name = Actions.Send

    def handle(self, command):
        self.usb.send(command.pack())
        Events.do(Actions.Log, command)

class ReadHandler:
    def __init__(self, handlers):
        self.name = Actions.Read
        self.handlers = handlers

    def handle(self, packet):
        while(len(packet) > 0):
            response, packet = self._unpack(packet)
            Events.do(Actions.Log, response)
            Events.do(Actions.Update, response)

    def _unpack(self, packet):
        for handler in self.handlers:
            if handler.can_handle(packet):
                response = handler(packet)
                return (response, packet[response.len:])

        Events.do(Actions.Log, "No handler for packet: %s" % str(packet))
        return (None, [])

class LogHandler:
    def __init__(self, logger):
        self.logger = logger
        self.name = Actions.Log

    def handle(self, obj_to_show):
        text = str(obj_to_show)
        if isinstance(obj_to_show, commands.NullUnpack) and text == "":
            return
        self.logger.append(text.rstrip('\n') + '\n')

class UpdateHandler:
    def __init__(self):
        self.name = Actions.Update

    def handle(self, unpacked):
        pass


class Sender:
    def send(self):
        Events.do(Actions.Send, self)

for p in commands.Command.packers:
    if Sender not in p.__bases__:
        p.__bases__ += (Sender,)

if __name__ == "__main__":
    print "Hello World"
