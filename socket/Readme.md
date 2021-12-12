1. On Line 14 of HR-Client05-SendBinary.cpp, there is this line

   >># define SERVER_IP  "192.168.109.121"<<

   Change "192.168.109.121" to the LAN-IP of machine B (the server)

2. Compile and run HR-Server05-ReceiveBinary.cpp on machine B (the server)

   Compilation : >>g++ HR-Server05-ReceiveBinary.cpp -o HR-Server05-ReceiveBinary<<

   Execution :   >>./HR-Server05-ReceiveBinary Dummy receivedImage.jpg<<

3. Compile and run HR-Client05-SendBinary.cpp on machine A (the client)

   Compilation : >>g++ HR-Client05-SendBinary.cpp -o HR-Client05-SendBinary<<

   Execution :   >>./HR-Client05-SendBinary file1.jpg Dummy<<

Note : the server (program) must be in execution before the client (program) is run
