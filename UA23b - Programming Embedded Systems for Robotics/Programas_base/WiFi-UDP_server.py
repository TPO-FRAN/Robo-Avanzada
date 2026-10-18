import socket

# data size for the communication channel
bufferSize =1024
# just a message to send back to a basic client
msgFromServer = 'Hello World, from the Server'
# port from where the server will listen to a client
serverPort = 2222
# IP Address assigned from router.
# Use hostname -I or /sbin/ifconfig to obtain the IP address
# of the client after it is connected to the modem
serverIP ='192.168.1.119'
# Encrypt the message to send to client with the standard utf-8
bytesToSend = msgFromServer.encode('utf-8')
# Prepare the socket variable
RPIsocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
RPIsocket.bind((serverIP, serverPort))
# At this point, the server can be up and listening
print('Server is UP and Listening...')
# Receive an income messsage from a client
message,address = RPIsocket.recvfrom(bufferSize)
# Decode the message information
message=message.decode('utf-8')
# Print the message & some extra client data
print(message)
print('Client Address', address[0])
RPIsocket.sendto(bytesToSend, address)
