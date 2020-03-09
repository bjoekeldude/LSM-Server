#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include <string>
#include <string_view>
#include <array>
#include <fstream>
#include <boost/filesystem.hpp>
#include <tuple>
#include <boost/core/ignore_unused.hpp>


namespace lsm{
    namespace fs=boost::filesystem;
    using command_t = std::tuple<std::string, int>;
    
    //magic constant. could also be larger but needn't to
    //be in this case
    constexpr int maxInputBufferLength{1024};
    std::string_view epicsProtocolTerminator="X";
    
    //encapsulates an instance of adc hardware on BBB
    //could be templated to be generic for different 
    //hardware.
    struct adc
    {
    private:
        fs::path sysfspath_;
        std::string egu_;
        int scalingFactor_;
        std::fstream adcHandle_;
        void printValue(int val)
        {
            std::clog << "ADC says " << val << " " << egu_ << std::endl;
        }
    public:
        adc(fs::path path, int scalingFactor=1, std::string egu="counts")
            :   sysfspath_(path), scalingFactor_(scalingFactor), egu_(egu)
        {
            adcHandle_.open(sysfspath_.c_str(), std::fstream::in);
            printValue(getRAWValue());
        }
        unsigned int getRAWValue()
        {
            std::string val;
            adcHandle_.seekg(0);
            std::getline(adcHandle_,val);
            unsigned int rval = std::stoi(val);
            return rval;
        }
    };

    //encapsulates an instance of PWM hardware on BBB
    //could also be templatized in order to be more generic
    struct pwm
    {
    private:
        int periodeVal_;
        fs::path sysfspath_;
        fs::path Capture_;
        std::ofstream CaptureHandle_;
        fs::path DutyCycle_;
        std::ofstream DutyCycleHandle_;
        fs::path Enable_;
        std::ofstream EnableHandle_;
        fs::path Period_;
        std::ofstream PeriodHandle_;
        fs::path Polarity_;
        std::ofstream PolarityHandle_;
        fs::path Uevent_;
        std::ofstream UeventHandle_;
        
        //makes sure one doesn't forget to flush
        void setProperty(std::ofstream& property, int val)
        {
            property << val;
            property.flush();
        }

    public:
        pwm(fs::path pwmPath, int periode = 100000000)
           : sysfspath_(pwmPath), periodeVal_(periode)
        {
            //opens filehandles and doesn't free them until deconstruction
            DutyCycleHandle_.open   (DutyCycle_ .c_str(),   std::fstream::out);
            PeriodHandle_   .open   (Period_    .c_str(),   std::fstream::out);
            PolarityHandle_ .open   (Polarity_  .c_str(),   std::fstream::out);
            EnableHandle_   .open   (Enable_    .c_str(),   std::fstream::out);
            
            //initialize as turned on, but frozen in position
            setProperty(DutyCycleHandle_,0);
            setProperty(PeriodHandle_,periodeVal_);
            setProperty(PolarityHandle_,0);
            setProperty(EnableHandle_,1);            
        }

        //there shouldn't be a need to access other parameters
        //than direction and velocity. Therfore members are not
        //exposed at all. 
        void update(int direction, int velocityPerMille)
        {
            setProperty(DutyCycleHandle_,0);
            int duty_cycleVal = (periodeVal_ / 1000) * velocityPerMille;
            setProperty(PolarityHandle_,direction);
            setProperty(DutyCycleHandle_,duty_cycleVal);
        }

    };

    //this struct encapsulates a generic PID controller that 
    //comes with two bounds. One could argue that this could
    //be a specialization of a templated controller. In order
    //to not let this project go down in code-bloat it was
    //decided to not generalize this implementation.
    //Might be a good idea to put this into a controller 
    //library, together with pure virtual interface 
    //classes for its ADC- and PWM-Handles.
    struct controller
    {
    private:
        int Kp_, Ki_, Kd_;
        adc& adcHandle_;
        pwm& pwmHandle_;
        unsigned int innerEndPosition_, outerEndPosition_;
        int integral_;
        int previousError_;
        int dt_;
        int setpoint_;

        bool isOccupied_ { false };

        boost::asio::steady_timer m_timer;
        std::chrono::milliseconds m_duration;
        std::function<void (const std::string&)> ackFunction_; 

        int calculateMeasuredError(unsigned int setpoint, int reference)
        {
            return (setpoint - reference);
        }
        int calculateProportionalTerm(int error)
        {
            return (error * Kp_);
        }
        int calculateIntegralTerm(int error)
        {
            integral_ += error * dt_;
            return (integral_ * Ki_);
        }
        int calculateDerivativeTerm(int error)
        {
            int derivative = (error - previousError_) / dt_;
            return (derivative * Kd_);
        }
        int calculateSystemInput(int measuredError)
        {
            return (    calculateProportionalTerm(measuredError)
                    +   calculateIntegralTerm(measuredError)
                    +   calculateDerivativeTerm(measuredError));
        }
        int decideDirection(int systemInput)
        {
            if(0<systemInput)return 1;
            else return 0;
        }

    public:
        controller(boost::asio::io_context& context, int p, int i, int d, adc& adcHandle, pwm& pwmHandle, unsigned int innerEndPosition, unsigned int outerEndPosition, unsigned int dt)
           :    Kp_(p), 
                Ki_(i), 
                Kd_(d),
                adcHandle_(adcHandle),
                pwmHandle_(pwmHandle),
                innerEndPosition_(innerEndPosition),
                outerEndPosition_(outerEndPosition),
                integral_(0),
                previousError_(0),
                dt_(dt),
                m_timer(context),
                m_duration(dt)
        {
            std::clog   << "Controller initialized with P="
                        << Kp_ << ", I=" << Ki_ << ", D=" << Kd_ 
                        << std::endl << "Iteration-Time is= " << dt
                        << std::endl;
        }

        int iterateToSetpoint(unsigned int setpoint)
        {
            int reference = static_cast<int>(adcHandle_.getRAWValue());
            int measuredError = calculateMeasuredError(setpoint, reference);
            int systemInput = calculateSystemInput(measuredError);
            int direction = decideDirection(systemInput);
            pwmHandle_.update(direction, systemInput);
            
            return measuredError;
        }
        int start(command_t command, std::function<void(const std::string&)>&& returnVal)
        {
            ackFunction_ = std::move(returnVal);
            if(isOccupied_) return -1;
            setpoint_ = std::get<1>(command);
            int currentPosition = iterateToSetpoint(setpoint_);
            if(100<currentPosition){
                isOccupied_ = true;
                start();
            }
            return adcHandle_.getRAWValue();
            
        }

        int start() {
            m_timer.expires_after(m_duration);
            m_timer.async_wait([this](const boost::system::error_code& error){
                if(!error ) {
                    int currentPosition = iterateToSetpoint(setpoint_); 
                    if (100<currentPosition)
                        start();
                        
                    else{
                        isOccupied_ = false;
                        std::string message = "Done " + std::to_string(currentPosition);
                        if(ackFunction_)ackFunction_(message);
                        
                    }
                } 
            });
            return adcHandle_.getRAWValue();
        }

        bool isOccupied() {
            return isOccupied_;
        }
    };
    
    //session is the organizer, responsible for parsing 
    //incoming messages and calling the controller
    //as well as handling the busy-state.   
    struct session
    : public std::enable_shared_from_this<session>
    {
        using bait = boost::asio::ip::tcp;
    public: 
        session(bait::tcp::socket socket, std::shared_ptr<controller> controllerInstance)
            : socket_(std::move(socket)), controller_(controllerInstance)
        {
        }

        void start(){
            doRead();
        }

    private:
        command_t makeCommandFromEpicsProtocol(std::array<char, 1024> inputData){
            std::string inputString(std::begin(inputData), std::end(inputData));
            std::string commandostring {inputString.erase(inputString.find(epicsProtocolTerminator))};
            return std::make_tuple(commandostring,1);
        }
        void acknowledge(std::string ackmessage){
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(ackmessage),
                    [this, self](boost::system::error_code ec, std::size_t /*length*/)
                    {
                        if(ec.value()){
                            std::cerr << ec.category().name() << ':' << ec.value() << "\n";
                        }
                    });
        }

        void doWrite(std::size_t length){
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(data_, length),
                    [this, self](boost::system::error_code ec, std::size_t /*length*/)
            {
                if(!ec){
                    doRead();
                }
            });
        }

        void doRead(){
            auto self(shared_from_this());
            socket_.async_read_some(boost::asio::buffer(data_),
                    [this, self](boost::system::error_code ec, std::size_t length)
            {
                boost::ignore_unused(length);
                command_t command = makeCommandFromEpicsProtocol(data_);
                if (controller_->isOccupied()) {
                    if(!ec){
                        acknowledge("Roger that\n");
                        controller_->start(command, [this,self](const std::string& message)
                                                            {acknowledge(message);});
                        
                    }

                }
                    else
                    acknowledge("I am occupied\n");
            });
        }

        //enum status;
    
        bait::tcp::socket socket_;
        std::shared_ptr<controller> controller_;
        std::array<char, maxInputBufferLength> data_;
    };
}
