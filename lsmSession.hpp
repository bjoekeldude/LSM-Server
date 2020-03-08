#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include <string>
#include <array>
#include <fstream>
#include <filesystem>


namespace lsm{
    namespace fs=std::filesystem;
    typedef enum{
        idle,
        busy,
        error
    } state;
    constexpr int maxInputBufferLength{1024};
    std::string epicsProtocolTerminator{"X"};
      
    struct adc
    {
    private:
        fs::path sysfspath_;
        std::string egu_;
        int scalingFactor_;
        std::fstream adcHandle_;
        void printValue(int val)
        {
            std::cout << "ADC says " << val << " " << egu_ << std::endl;
        }
    public:
        adc(fs::path path, int scalingFactor=1, std::string egu="counts")
            :   sysfspath_(path), scalingFactor_(scalingFactor), egu_(egu)
        {
            adcHandle_.open(sysfspath_, std::fstream::in);
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
        void setProperty(std::ofstream& property, int val)
        {
            property << val;
            property.flush();
        }
    public:
        pwm(fs::path pwmPath, int periode = 100000000)
           : sysfspath_(pwmPath), periodeVal_(periode)
        {
            DutyCycleHandle_.open   (DutyCycle_,    std::fstream::out);
            PeriodHandle_.open      (Period_,       std::fstream::out);
            PolarityHandle_.open    (Polarity_,     std::fstream::out);
            EnableHandle_.open      (Enable_,       std::fstream::out);
            
            setProperty(DutyCycleHandle_,0);
            setProperty(PeriodHandle_,periodeVal_);
            setProperty(PolarityHandle_,0);
            setProperty(EnableHandle_,1);            
        }

        void update(int direction, int velocityPerMille)
        {
            setProperty(DutyCycleHandle_,0);
            int duty_cycleVal = (periodeVal_ / 1000) * velocityPerMille;
            setProperty(PolarityHandle_,direction);
            setProperty(DutyCycleHandle_,duty_cycleVal);
        }

    };

    struct controller
    {
    private:
        int Kp_, Ki_, Kd_;
        adc& adcHandle_;
        pwm& pwmHandle_;
        unsigned int innerEndPosition_, outerEndPosition_;
        int integral_;
        int preError_;
        int dt_;
    public:
        controller(int dt, int p, int i, int d, adc& adcHandle, pwm& pwmHandle, unsigned int innerEndPosition, unsigned int outerEndPosition)
           :    Kp_(p), 
                Ki_(i), 
                Kd_(d),
                adcHandle_(adcHandle),
                pwmHandle_(pwmHandle),
                innerEndPosition_(innerEndPosition),
                outerEndPosition_(outerEndPosition),
                integral_(0),
                preError_(0),
                dt_(dt)
        {
            std::clog   << "Controller initialized with P="
                        << Kp_ << ", I=" << Ki_ << ", D=" << Kd_ << std::endl;
        }
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
            int derivative = (error - preError_) / dt_;
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

        int runToSetpoint(unsigned int setpoint)
        {
            int reference = adcHandle_.getRAWValue();
            int measuredError = calculateMeasuredError(setpoint, reference);
            if(100<measuredError)
            {
                int systemInput = calculateSystemInput(measuredError);
                int direction = decideDirection(systemInput);
                pwmHandle_.update(direction, systemInput);
                //wait
                //start over again
            }
            else return measuredError;
        }
    };

    struct session
    : public std::enable_shared_from_this<session>
    {
        using bait = boost::asio::ip::tcp;
        using command_t = std::string;

    public: 
        session(bait::tcp::socket socket)
            : socket_(std::move(socket))
        {
        }

        void start(){
            doRead();
        }

    private:
        command_t makeCommandFromEpicsProtocol(std::array<char, 1024> inputData){
            std::string inputString(std::begin(inputData), std::end(inputData));
            return command_t{inputString.erase(inputString.find(epicsProtocolTerminator))};
        }

        void actOutCommand(command_t command){

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
                makeCommandFromEpicsProtocol(data_);
                if(!ec){
                    acknowledge("Roger that\n");
                }
            });
        }
        //enum status;
        bait::tcp::socket socket_;
        std::array<char, maxInputBufferLength> data_;
    };
}