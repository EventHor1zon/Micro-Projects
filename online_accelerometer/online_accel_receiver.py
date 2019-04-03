#!/usr/bin/python

import argparse
import socket
import struct
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.style as style



def main():

    HOST="0.0.0.0"
    PORT=None
    LOG=False
    LOG_PATH=""
    PARAMETER="aX"
    GRAPH=True
    data_options=['aX', 'aY', 'aZ', 'daX', 'daY', 'daZ', 'gX', 'gY', 'gZ', 'dgX', 'dgY', 'dgZ']
    last_parsed = (None,None,None,None,None,None,None,None,None,None)

    # argparsing
    parser=argparse.ArgumentParser()
    parser.add_argument("-p", dest="PORT", type=int, help="The local port to listen on", default=3333)
    parser.add_argument("-l", dest="log", action="store_true", help="Log data to a local file (requires -o)")
    parser.add_argument("-o", dest="output", type=str, help="output log file")
    parser.add_argument("-g", dest="graph", action="store_true", help="Graph the received data live, requires -D")
    parser.add_argument("-D", dest="data", type=str, help="Data to live graph: choose 'aX', 'aY', 'aZ', 'daX', 'daY', 'daZ', etc ")
    args=parser.parse_args()

    if args.PORT:
        PORT = args.PORT
    if args.log and not args.output:
        print("Must specify an output file with -o")
        sys.exit(1)
    elif args.log and args.output:
        LOG=True
        LOG_PATH = args.output
    if args.graph and not args.data:
        print("Please choose one of the data types to log: ")
        print(",".join(data_options))
        sys.exit(0)
    elif args.graph and args.data:
        if args.data not in data_options:
            print("Error: selected data not recognised")
            sys.exit(1)
        else:
            # build graph
            PARAMETER = args.data
            GRAPH = True
            ani = animation.FuncAnimation(fig, animate, interval=50)

    if LOG:
        try:
            f=open(LOG_PATH+"log_ax", "w")
        except:
            print("Error opening file at ", LOG_PATH)
            sys.exit(1)



    sock=None
    # sock stream
    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        print("Opening socket on port: ", str(PORT))
        sock.bind((HOST,PORT))
        sock.listen(1)
    except socket.error as err:
        sock.close()
        print(err)
        sys.exit()

    connection,address=sock.accept()

    print("Connection from: ", address)

    try:
        while True:
            data=connection.recv(256)
            if len(data) > 26:
                # unexpected packet size
                print("error: data length = ", str(len(data)))
                pass
            elif len(data)== 0:
                # empty packet or broken pipe
                print("error: no more data to receive?")
            elif len(data) < 26:
                try:
                    # heartbeat signal
                    decoded = data.decode('utf-8')
                    if "HEART" in decoded:
                        send(connection, "BEAT")
                except:
                    #unexpected packet size or unexpected characters
                    pass
            else:
                #packet size expected,unpack contents
                aX,aY,aZ,gX,gY,gZ,temp,id,timestamp,delta_t=parse(data)
                parsed=(aX,aY,aZ,gX,gY,gZ,temp,id,timestamp,delta_t)
                if GRAPH:
                    plot(PARAMETER, parsed, last_parsed)
                    ax1.plot(x_data, y_data)
                if LOG:
                    log(parsed)
                last_parsed = parsed

    except KeyboardInterrupt:
        connection.close()
        print("Exitting... Goodbye!")
        sys.exit(0)

def animate(i):
    ax1.plot(x_data, y_data)
    plt.show()

def plot(PARAMETER, parsed, last_parsed):

    if PARAMETER == "aX":
        graph(parsed[0], parsed[8])
    elif PARAMETER == "aY":
        graph(parsed[1], parsed[8])
    elif PARAMETER == "aZ":
        graph(parsed[2], parsed[8])
    elif PARAMETER == "gX":
        graph(parsed[3], parsed[8])
    elif PARAMETER == "gY":
        graph(parsed[4], parsed[8])
    elif PARAMETER == "gZ":
        graph(parsed[5], parsed[8])
    elif PARAMETER == "temp":
        graph(parsed[6], parsed[8])
    elif PARAMETER == "daX":
        graph(delta(last_parsed[0], parsed[0], parsed[9]), parsed[8])
    elif PARAMETER == "daY":
        graph(delta(last_parsed[1], parsed[1], parsed[9]), parsed[8])
    elif PARAMETER == "daZ":
        graph(delta(last_parsed[2], parsed[2], parsed[9]), parsed[8])
    elif PARAMETER == "dgX":
        graph(delta(last_parsed[3], parsed[3], parsed[9]), parsed[8])
    elif PARAMETER == "dgY":
        graph(delta(last_parsed[4], parsed[4], parsed[9]), parsed[8])
    elif PARAMETER == "dgZ":
        graph(delta(last_parsed[5], parsed[5], parsed[9]), parsed[8])
    else:
        print("Error in PARAMETER selection")
        sys.exit(1)

def delta(val_prev, val, dT):
    if val_prev == None:
        return None
    else:
        change = (val - val_prev) / dT
    return change

def graph(x, y):
    x_data.append(x)
    y_data.append(y)

def open_socket():
    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    return sock



def parse(data):
    #   struct MPU = uint16_t *6: (aX->gZ)
    #                double: temp
    #                uint16_t: id
    #                uint32_t * 2: timestamp, delta_t
    #
    #
    #
    aX,aY,aZ,gX,gY,gZ,temp,id,timestamp,delta_t=struct.unpack('=hhhhhhfHLL', data)
    print(aX,aY,aZ,gX,gY,gZ,temp,id)
    return aX,aY,aZ,gX,gY,gZ,temp,id,timestamp,delta_t



if __name__ == '__main__':
    main()
