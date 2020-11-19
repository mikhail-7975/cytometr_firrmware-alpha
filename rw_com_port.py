import serial
import time
counter_test = 0

trigger_port = None
tracer1_port = None


def write_trigLevel_gain(port: str, triggerLevel: str, gain: str):
    global counter_test, trigger_port
    counter_test += 1
    _port = port.replace(" ", "")
    _triggerLevel = triggerLevel.replace(" ", "").rjust(4, '0')
    _gain = gain.replace(" ", "").rjust(4, '0')
    message = b't' + _triggerLevel.encode('utf-8') + b'.g' + _gain.encode('utf-8') + b'.'
    print(message)
    try:
        if (trigger_port == None):
            trigger_port = serial.Serial(_port)
        trigger_port.write_timeout = 1  # Set write timeout, sec
        trigger_port.write(message)
        return "ok"
    except serial.SerialTimeoutException:
        print("TimeoutException")
        return "TimeoutException"
    except serial.SerialException:
        print("SerialException")
        return "SerialException" + str(counter_test)
    except Exception:
        print("something wrong")
        return "something wrong"

def write_gain(port: str, gain: str):
    global counter_test, tracer1_port
    counter_test += 1
    _port = port.replace(" ", "")
    _triggerLevel = "0".replace(" ", "").rjust(4, '0')
    _gain = gain.replace(" ", "").rjust(4, '0')
    message = b't' + _triggerLevel.encode('utf-8') + b'.g' + _gain.encode('utf-8') + b'.'
    print(message)
    try:
        if (tracer1_port == None):
            tracer1_port = serial.Serial(_port)
        tracer1_port.write_timeout = 1 #Set write timeout, sec
        tracer1_port.write(message)
        return "ok"
    except serial.SerialTimeoutException:
        print("TimeoutException")
        return "TimeoutException"
    except serial.SerialException:
        print("SerialException")
        return "SerialException"  + str(counter_test)
    except Exception:
        print("something wrong")
        return "something wrong"


def read_data_from_port(port):
    data = []

    while len(data) < 1002:
        try:
            port.write(b"req")  # <====================
        except serial.serialutil.SerialTimeoutException:
            print("time over")
        #print("send request...")

        startTime = time.time()
        while not port.inWaiting():
            # print('.')
            if time.time() - startTime > 1:
                #print("much time not in waiting")
                break

        i = 0
        inp = 0
        startTime = time.time()
        while (inp != 254):
            while port.inWaiting():
                # print(ord(port.read()), i)
                inp1 = port.read()
                inp2 = port.read()
                data = ord(inp2) << 8 | ord(inp1)
                if data > 65000:
                    list.append(data)
                    break
                list.append(data)
                # print("reading", i)
                i += 1
            if time.time() - startTime > 1:
                print("much time in trying to read")
                break

        try:
            if data[0] == 255 and data[-1] == 254 and len(data) == 1002:
                break
            else:
                #print("error", "start =", data[0] == 255, "stop =", data[-1] == 254, "len =", len(data))
                data = []
        except Exception:
            print(Exception)
            # a = 0

    port.write(b"ack")
    return data


def read_data_from_all_ports():
    global tracer1_port, trigger_port
    trigger = read_data_from_port(trigger_port)
    tracer1 = read_data_from_port(tracer1_port)
    return [trigger, tracer1]



def read_data_from_tracer1():
    data = []
    global tracer1_port
    while len(data) < 5002:
        try:
            tracer1_port.write(b"req")  # <====================
        except serial.serialutil.SerialTimeoutException:
            print("time over")
        # print("send request...")

        startTime = time.time()
        while not tracer1_port.inWaiting():
            # print('.')
            if time.time() - startTime > 1:
                # print("much time not in waiting")
                break

        i = 0
        inp = 0
        startTime = time.time()
        while (inp != 254):
            while tracer1_port.inWaiting():
                # print(ord(port.read()), i)
                inp = ord(tracer1_port.read())
                data.append(inp)
                # print("reading", i)
                i += 1
            if time.time() - startTime > 1:
                print("much time in trying to read")
                break

        try:
            if data[0] == 255 and data[-1] == 254 and len(data) == 1002:
                break
            else:
                # print("error", "start =", data[0] == 255, "stop =", data[-1] == 254, "len =", len(data))
                data = []
        except Exception:
            print(Exception)
    return []

def read_data_from_trigger():
    return []

def set_tracer1_port(port):
    global tracer1_port
    tracer1_port = serial.Serial(port)
    return 0

def set_tracer1_gain(gain):
    global counter_test, trigger_port
    counter_test += 1
    _gain = gain.replace(" ", "").rjust(4, '0')
    message = (chr(1) + _gain).encode('utf-8')
    print(message)
    try:
        if (trigger_port == None):
            return 0
        trigger_port.write_timeout = 1  # Set write timeout, sec
        trigger_port.write(message)
        return "ok"
    except serial.SerialTimeoutException:
        print("TimeoutException")
        return "TimeoutException"
    except serial.SerialException:
        print("SerialException")
        return "SerialException" + str(counter_test)
    except Exception:
        print("something wrong")
        return "something wrong"
    return 0

def set_trigger_com_port(port):
    global trigger_port
    trigger_port = serial.Serial(port)
    return 0

def set_trigger_gain(gain):
    global counter_test, trigger_port
    counter_test += 1
    _gain = gain.replace(" ", "").rjust(4, '0')
    message = (chr(1) + _gain).encode('utf-8')
    print(message)
    try:
        if (trigger_port == None):
            return 0
        trigger_port.write_timeout = 1  # Set write timeout, sec
        trigger_port.write(message)
        return "ok"
    except serial.SerialTimeoutException:
        print("TimeoutException")
        return "TimeoutException"
    except serial.SerialException:
        print("SerialException")
        return "SerialException" + str(counter_test)
    except Exception:
        print("something wrong")
        return "something wrong"
    return 0

def set_trigger_level(level):
    global counter_test, trigger_port
    counter_test += 1
    _level = level.replace(" ", "").rjust(4, '0')
    message = (chr(1) + _level).encode('utf-8')
    print(message)
    try:
        if (trigger_port == None):
            return 0
        trigger_port.write_timeout = 1  # Set write timeout, sec
        trigger_port.write(message)
        return "ok"
    except serial.SerialTimeoutException:
        print("TimeoutException")
        return "TimeoutException"
    except serial.SerialException:
        print("SerialException")
        return "SerialException" + str(counter_test)
    except Exception:
        print("something wrong")
        return "something wrong"
    return 0
#150 * 2 + 2 * 2 - размер триггера
#длина - 152

#5000 * 2 + 2 * 2 - размер трейсера
#длина - 5002

#расстояние от ввода до триггера -
import matplotlib.pyplot as plt

port = serial.Serial('COM5')
port.write(b'ack')
len = 2000
while(1):


    while(not port.inWaiting()):
        print('.')

    print("begin reading")
    buf_trace = port.read(len * 2)
    print(buf_trace)

    trace = []
    for i in range(len):
        trace.append(buf_trace[2*i] + buf_trace[2*i+1]*256)

    print(trace)
    plt.plot(trace, 'x')
    plt.show()
    port.write(b'ack')
    print("wrote ack")
