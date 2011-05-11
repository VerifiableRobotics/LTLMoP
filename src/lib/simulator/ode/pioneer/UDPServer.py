import SocketServer, time
from multiprocessing import Process

command = "SServer%(0,0)"
responseODE = ""

class MyUDPHandler(SocketServer.BaseRequestHandler):
    """
    Handle the communication between LTLMOP and ODE_SIMULATOR
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        #global variable command, used by LTLMOP client to set velocity of wheels.
        global command
        #global variable responseODE, used by ODE to send POSE information back to LTLMOP
        global responseODE
        # Cleanning with spaces
        
        data = self.request[0].strip()
        # Getting the socket
        socket = self.request[1] 

        #Cleanning information from a messy string
        mess = data.split('%') 
        
        #Identitying the origin of the message evoiding the use of two ports communication
        orig = mess[0]  
        if (orig == 'ODE'):
            responseODE = mess[1] #If the message comes from ODE set global 'responseODE' variable. By now only send the POSE information.
        if (orig == 'LTLMOP'):
            if mess[1] != "ignore":
                command = "SServer%" + mess[1] #If the message comes from LTLMOP set global 'command' variable

            socket.sendto(responseODE, self.client_address) #Inmediatly send to ODE
            
        socket.sendto(command, self.client_address) #Always sending message back
##        print "FROMUDP SERVER:",command
        

if __name__ == "__main__":
    HOST, PORT = "localhost", 23456 
    server = SocketServer.UDPServer((HOST, PORT), MyUDPHandler) 
##    server.serve_forever() #Keep listening UDP Datagrams.

        
    p = Process(target=server.serve_forever())
    p.start()
    
