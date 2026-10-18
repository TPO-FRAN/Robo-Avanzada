import socket

# just a message to send to a basic server
msgFromClient = 'Hello World, from your client'
# Encrypt the message to send to client with the standard utf-8
bytesToSend = msgFromClient.encode('utf-8')
# Server IP Address assigned from router.
serverAddress = ('192.168.0.12', 2222)

# data size for the communication channel
bufferSize = 1024
# Prepare the client socket variable
UDPClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Send client's encrypted message to serverAddress
UDPClient.sendto(bytesToSend, serverAddress)
# Receive an incoming messsage from Server
data, address = UDPClient.recvfrom(bufferSize)
# Decode the message information
data = data.decode('utf-8')
# Print the message & some extra server data
print('Data from Server: ', data)
print('Server IP Address: ', address[0])
print('Server Port: ', address[1])
