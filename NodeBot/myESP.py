# some general useful functions 
# for use in whatever projects

# Pinmappings
#D3 = 0
#D4 = 2 # (also Led1 but inverse)*
#D2 = 4
#D1 = 5
#SD2 = 9
#SD3 = 10
#D6 = 12
#D7 = 13
#D5 = 14
#D8 = 15
#D0 = 16 # (also Led2 but inverse)*

import network
import socket
import ssd1306
from time import sleep, sleep_ms
from machine import Pin, I2C

def print_connect_info(deets):
    print("Connection Details:")
    st="IP: "+str(deets[0]) + "\tSubnet: "+str(deets[1])+"\nGateway: "+str(deets[2])+"\tDNS: "+str(deets[3])
    print(st)
    return


def getCommand(self, line):
    # splits command from line request and sends for parsing
    valid_commands=["drive", "network", "lights", "echo", "beep"]
    parts=line.split("&")                   # split by args
    parts[-1]=parts[-1].split(" ")[0]       # get rid of HTTP 1.1/ etc...
    command=parts[0].split("=")[1]
    args=[]
    print("Got request including...")
    print(command)
    if len(parts) > 2:   # pick up optional arg 1
        arg1=parts[1].split("=")[1]
    else:
        arg1=None
    if len(parts) > 3:   # arg2
        arg2=parts[2].split("=")[1]
    else:
        arg2=None
    if len(parts) > 4:   # and arg3
        arg3=parts[3].split("=")[1]
    else:
        arg3=None
    args.append(arg1)
    args.append(arg2)
    args.append(arg3)
    print("***DEBUG***")
    print(args)
    print(command)
    if command not in valid_commands:   # check for invalid command
        print("Invalid command recieved! ", command)
        return
    if command == "drive":          # drive commands
    #    drive(args)   # only requires 1 argument
    #elif command == "network":      # network commands
    #    networking(args)
    #elif command == "echo":         # print a line
    #    echo(args)
    #elif command == "lights":       # lighting commands
    #    lights(args)
    #else:
    #    return
    return

def listen_for_command():
    # listens for requests to arrive, sends them onto getCommand
    rp="""<!DOCTYPE HTML>
    <html>
        <head> <title> ESP </title> </head>
        <body> <p> Hi! </p> </body>
    </html>"""
    addr=socket.getaddrinfo("0.0.0.0", 80)[0][-1]
    sock=socket.socket()
    sock.bind(addr)
    sock.listen(1)
    print("Listening on ", addr)
    response="Ok!\r\n"
    while(True):    # opens a file-socket object (learn more?) to read from
        cl,addr=sock.accept()
        print("client connected from ", addr)
        cl_file=cl.makefile('rwb', 0)
        while True:
            line=cl_file.readline()
            if not line or line == b'\r\n':
                break
            elif "command" in line:
                getCommand(line.decode())   # remember to use line.decode to send string!
            else:
                print(line.decode())
        cl.send(rp)
        print("Closing connection...")
        cl.close()  # remember to close or memory becomes an issue

def connect_network(NETWORK, PASS):
    # connect to the network specified using network and pass
    radio=network.WLAN(network.STA_IF)
    radio.active(True)
    err_c=0
    while not radio.isconnected():              # begin network connection
        radio.connect(NETWORK, PASS)
        sleep(2)
        err_c=err_c+1
        if err_c > 10:
            print("Unable to connect...")
            return False
    print("Connected!")
    radio.ifconfig()
    return True

def connect_network_old(ssid, passwd):
    import network
    from time import sleep
    n=network.WLAN(network.STA_IF)
    n.active(True)
    if ssid not in [str(x[0]).lstrip("b").strip("'") for x in n.scan()]:    # i can comprehend list!
        print("AP not in scan range")
        return None
    try:
        n.connect(ssid, passwd)         # connect
        while n.status() == 255:            # while unconnected
            print("*", end="")
            sleep(0.2)                  # wait
        s=n.status()                    # get status
        if s == 5:                      # if connected, yay.
            print("Connected!")
            print_connect_info(n.ifconfig())
            return n                    # return network handle
        elif s == 2:                    # pass fail
            print("Incorrect Password!")
            raise RuntimeError("PASSWD_FAIL")
        elif s == 3:                    # AP fail
            print("AP did not reply *sadface*")
            raise RuntimeError("AP_FAIL")
        elif s == 4:                    # other fails
            print("Connection failed in other, mysterious ways.")
            raise RuntimeError("UNKNOWN_EXCEPTION")
    except RuntimeError as e:           # handle error
        n.active(False)                 # turn off the interface, prevent endless reconnect attempts
        print(e)                   # print error handle
        return None                     # return

def http_post_data(data, addr, port):
    import socket
    s=socket.socket()
    s.connect((addr, port))
    LF="\r\n"
    LE=LF+LF
    sendString="POST / HTTP/1.0" + LF
    sendString+="Host: homepi" + LF
    sendString+="User-agent: GardenNode"
    sendString+="Content-type: text/html" + LF
    sendString+="Content-Length: "+str(len(data))+LF
    sendString+=LF
    sendString+=data
    sendString+=LE
    s.send(bytes(sendString))
    while(1):
        reply=s.recv(500)
        if reply:
            print(reply)
        else:
            break
    s.close()

def init_i2c(scl, sda):
    from machine import Pin, I2C
    try:
        i2c=I2C(-1, Pin(scl), Pin(sda))
        return i2c
    except:
        print("Error Initialising I2C")
        return None


def start_screen(SC_WIDTH, SC_HEIGHT, i2c):
    screen=ssd1306.SSD1306_I2C(SC_WIDTH, SC_HEIGHT, i2c)
    screen.fill(1)
    screen.show()
    sleep(0.5)
    screen.fill(0)
    screen.show()
    return screen


def sort_wl_scans(sc):
    # gives list of WiFi: (ssid, bssid, channel, RSSI, authmode, hidden).
    # split into human readable stuff. Aim to print on a single line so build string
    output=""
    output+=str(sc[0]).lstrip("b")          # add SSID
    output+="\t"
    #output+=str(sc[1]).strip("\'").lstrip("b").replace("\\x", ":").lstrip("\'").lstrip(":") #mac formatting
    output+=mac_print(sc[1])
    output+="\t"
    output+="Ch: "      # channel
    output+=str(sc[2])
    output+="\t"
    output+="Str: "     # strength
    output+=str(sc[3])
    output+="\t"
    output+="AUTH: "    # authmode (todo: update with specifics)
    output+=auth_type(sc[4])
    output+="\tHidden: "# hidden status
    if(int(sc[5]) > 0):
        output+="Yes"
    else:
        output+="No"
    print(output)

def mac_print(mac):
    out=""
    mac=str(ubinascii.hexlify(mac)).lstrip("b").strip("\'")
    for x in range(len(mac)):
        out+=mac[x]
        if x%2==0:      # yeah not quite right...
            out+=":"
    return out


def auth_type(auth):
    if auth == 0:
        return "OPEN"
    elif auth == 1:
        return "WEP"
    elif auth == 2:
        return "WPA-PSK"
    elif auth == 3:
        return "WPA2-PSK"
    elif auth == 4:
        return "WPA/WPA2-PSK"
    else:
        return "UNKNOWN"
        '''parts=split_line_every_nth(mac, 2)
        for x in range(len(parts)):
            out+=parts[x]
            out+=":"
        out.rstrip(":")
        return out'''

def print_TH(d, oled):
    # assume oled and ht setup  already.
    from time import sleep
    while(1):
        d.measure()
        t=d.temperature()
        h=d.humidity()
        t_str="Temperature: " + str(t) + "c"
        h_str="Humidity: " + str(h) + "%"
        oled.fill(0)
        oled.text(t_str, 0, 0)
        oled.text(h_str, 0, 10)
        oled.show()
        sleep(2)
