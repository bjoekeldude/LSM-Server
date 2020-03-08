target: asioLSMserver.cpp
	g++ asioLSMserver.cpp -std=c++2a -lpthread -lstdc++fs -o LSM-Server

clean:
	rm *.o*