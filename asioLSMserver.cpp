#include <cstdlib>
#include <boost/asio.hpp>
#include <exception>
#include <boost/filesystem.hpp>
#include <string>
#include "lsmSession.hpp"
using bait = boost::asio::ip::tcp;

//configuring the paths to sysfs
namespace config{
  namespace fs = boost::filesystem;
  fs::path systemsAdc		="/sys/bus/iio/devices/iio:device0/in_voltage0_raw";
  fs::path systemsPwm		="/sys/class/pwm/pwm-3:0/";
  constexpr int Kp{1};
  constexpr int Ki{1};
  constexpr int Kd{0};
  constexpr int controllerRefreshDelayMs{100};
  constexpr int innerEndpointKalibration{2700};
  constexpr int outerEndpointKalibration{100};
}
namespace error{
  class PortException
    : public std::exception
  {
    virtual const char* what() const throw()
    {
      return "Invalid Port Number - Port must be greater than 0 and less than 65535";
    }
  } wrongPortException;

  class InvokationException
    : public std::exception
  {
    virtual const char* what() const throw()
    {
      return "Illegal invokation - Try asioLSMserver <port to listen to>";
    }
  } illegalInvokationException;
}

//server launches a controller-session for incoming connections
//meanwhile it waits in doAccept() function.... idling
struct server{
public: 
    server(boost::asio::io_context& io_context, short port, std::shared_ptr<lsm::controller> controllerInstance)
		: acceptor_(io_context, bait::tcp::endpoint(bait::tcp::v4(), port)),
		socket_(std::make_shared<bait::tcp::socket>(io_context))
	{
		doAccept();
	}

private:
	void doAccept(){
		acceptor_.async_accept(*socket_,
			[this](boost::system::error_code ec)
		{
			if(!ec){
                std::make_shared<lsm::session>(socket_, controller)->start();
			}
			doAccept();
		});
	}
	bait::tcp::acceptor acceptor_;
	std::shared_ptr<bait::tcp::socket> socket_;
  std::shared_ptr<lsm::controller> controller;
};

int main(int argc, char* argv[]){

    if(argc != 2) throw error::illegalInvokationException;

    try{
        boost::asio::io_context io_context;
        lsm::adc adc(config::systemsAdc);
        lsm::pwm pwm(config::systemsPwm);
        auto controllerInstance = std::make_shared<lsm::controller>
                  (io_context,
                  config::Kp,
                  config::Ki,
                  config::Kd,
                  adc,
                  pwm,
                  config::innerEndpointKalibration,
                  config::outerEndpointKalibration, 
                  config::controllerRefreshDelayMs);
        int port{std::atoi(argv[1])};
        if(1>port || 65535<port) throw error::wrongPortException;
        server s(io_context, port, controllerInstance);
        io_context.run();
    }
    catch (std::exception& e){
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}
