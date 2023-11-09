#include <ros/ros.h>
#include <ros/master.h>
#include <ros/package.h>

#include <regex>
#include <unistd.h>
#include <wiringSerial.h>

#include "libraries/json.hpp"

#include <container/gps_msg.h>

using namespace ros;
using namespace std;

class GpsDriver {
    public:
        GpsDriver(NodeHandle nh_priv);
		~GpsDriver();

        void GpsFeedbackCallback(const TimerEvent& event);

    private:
        int fd, status;
        string node_name;

        struct module_data
		{
			int sim=0;
			int rssi=0,rssiDBM=0,rssiPercentage=0;
			double longitud=0;
			double latitud=0;
			double altitud=0;
			string latHemisphere;
			string lonHemisphere;
			string UTCTime = "";
			string UTCDate = "";
			double accuracy = 0;
			int satellites = 0;
			string iccid = "";    
			string provider = "";
			string tech = "";
			string band = "";
			bool latValid=false, longValid=false, gpsValid=false;
			vector<string> satellitesSNR,satellitesPRN,satellitesUsed, satelliteInfo;
		}data;

        struct module_dgnst
		{
			bool session = false;
			bool satellites = false;
			bool coordinates = false;
			bool sim = false;
			bool fixed = false;
		}dgnst;	

        NodeHandle nh;		
        Publisher GpsFeedbackPub;
        container::gps_msg gpsData;

        int InitializeModule();
        int DeinitializeModule();
        string SendATCommand(const char* command);
        void GetQuality();
        void SimInserted();
        void GetPosition();
        void GetUTCTime(string utc);
        void GetLatitud(string latitude);
		void GetLongitud(string longitud);
        void GetAccuracy(string hdop);
		void GetAltitud(string altitud);
        void GetUTCDate(string date);
        int GetSatellites(string satellites);
        void GetNetworkInformation();
};

GpsDriver::GpsDriver(NodeHandle nh_priv){
	node_name = this_node::getName();
    int log_level;
    nh_priv.param("log_level", log_level,0);	
	console::levels::Level console_level;
    console_level = (console::levels::Level)log_level;    
    if( console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level) ) {
        console::notifyLoggerLevelsChanged();
    }

    GpsFeedbackPub = nh.advertise<container::gps_msg>("sensor_topic/gps", 1);

    InitializeModule();
}

string GpsDriver::SendATCommand(const char* command){
    size_t cmdlen = strlen(command);
    ROS_DEBUG_STREAM("Command: " << command);

    int n = write(fd,command, cmdlen);
    if (n != cmdlen)
    {
      ROS_DEBUG_STREAM("Error in command length: " << n);
      return "";
    }
    Duration(0.3).sleep();
    char reply[1024];
    ROS_DEBUG_STREAM("Reading buffer...");
    n = read(fd,reply,sizeof(reply));
    reply[n] = '\0' ;
    string str(reply);
    /* vector<string> split = splitString(str,'\n');
    for(int i=0;i<split.size();i++)
    {
	if(split[i].size()==3)
	    ROS_DEBUG_STREAM(split[i]);
	if(split[i]=="OK\0")
	    ROS_DEBUG_STREAM(split[i]);
    } */
    ROS_DEBUG_STREAM("Response:" << str);
    return str;
}

int GpsDriver::InitializeModule(){
    fd = serialOpen("/dev/ttyUSB2",115200); 
    Duration(0.25).sleep();
    
    if(fd > 0){
        const char* command;
        string response;
        command = "AT+QGPS=1\r";	// Turn on GNSS engine
        response = SendATCommand(command);
        command = "AT+QSIMSTAT=1\r"; 	// Enable SIM Inserted Status Report
        response = SendATCommand(command);
        command = "AT+QGPSXTRA=0\r"; 	// Disable XTRA
        response = SendATCommand(command);
        status = 1;	
        return 1;
    }
    status = 0; //module not connected
    return 0;
}

int GpsDriver::DeinitializeModule(){
    const char* command = "AT+QGPSEND\r";
    string res = SendATCommand(command);
    close(fd);
    return 1;
}

void GpsDriver::GpsFeedbackCallback(const TimerEvent& event){
	GetQuality();
	if(data.sim < 0)
		data.rssi += 300;
	GetPosition();
	GetNetworkInformation();
		
	gpsData.validGPS=data.gpsValid;	
	gpsData.Latitude=data.latitud;
	gpsData.Longitude=data.longitud;
	gpsData.Altitude=data.altitud;
	gpsData.Sim=data.sim;
	gpsData.rssi=data.rssiPercentage;
	gpsData.LatHemisphere = data.latHemisphere;
	gpsData.LonHemisphere = data.lonHemisphere;
	gpsData.Accuracy = data.accuracy;
	gpsData.UTCTime = data.UTCTime;
	gpsData.UTCDate = data.UTCDate;
	gpsData.Satellites = data.satellites;
	gpsData.provider = data.provider;
	gpsData.accessTechnology = data.tech;
	gpsData.band = data.band;
	gpsData.ModuleConnected = fd;

	GpsFeedbackPub.publish(gpsData);
}

vector<string> split(string data, char spliter)
{
	vector<string> tokens;
	string temp;
	stringstream check1(data);
	while(getline(check1, temp, spliter))
	{
		tokens.push_back(temp);
	}
	return tokens;
}

void GpsDriver::SimInserted(){
    const char* command;
    string reply;
    
    command = "AT+QSIMSTAT?\r";
	reply = SendATCommand(command);
	if(reply.size() > 24)
    {
		try
		{
			vector<string> split_1 = split(reply,'\n');    
			vector<string> split_2 = split(split_1.at(1),' ');
			vector<string> split_3 = split(split_2.at(1),',');
			char inserted = split_3.at(1)[0];
			if(inserted == '1'){
				data.sim = 1;
			}
			else
			{
				data.sim = 0;
			}
		} catch(...){
			ROS_WARN_STREAM("SimInserted() error. Not fatal. Just ignore it.");
			data.sim = 0;
		}
	}
    else
		data.sim = 0;
}

void GpsDriver::GetQuality(){
    const char* command;
    string reply;
    
    command = "AT+CSQ\r";
    reply = SendATCommand("AT+CSQ\r");
    SimInserted();    
    if(data.sim > 0){
		if(reply.size() > 10)
		{
			try
			{
				vector<string> split_1 = split(reply,' ');
				vector<string> split_2 = split(split_1.at(1),',');
				data.rssi = stoi(split_2.at(0));
			}
			catch(...)
			{
				ROS_WARN_STREAM("GetQuality() error. Not fatal. Just ignore it.");
				data.rssi=0;
			}
		}
		else
			data.rssi = 200;
		}
		else
			data.rssi = 200; 
		
		if(data.rssi == 200)
		{
			data.rssiDBM=0;
			data.rssiPercentage=0;
		}
		else
		{
		
		float originalDBM = data.rssi;
		float aproxDBM = 0;
		if (data.rssi == 0)
			aproxDBM = -114;
		else if (data.rssi == 1)
			aproxDBM = -111;
		else if (data.rssi >= 2 && data.rssi <= 30)
			aproxDBM = (originalDBM - 2) / (30 - 2) * (-53 - (-109)) + (-109);
		else if (data.rssi >= 102 && data.rssi <= 190)
			aproxDBM = (data.rssi - 102) / (190 - 102) * (-26 - (-114)) + (-114);
		else if (data.rssi == 31)
			aproxDBM = -60;
		else if (data.rssi == 99)
			aproxDBM = -200;
		else if (data.rssi == 100)
			aproxDBM = -116;
		else if (data.rssi == 101)
			aproxDBM = -115;
		else if (data.rssi == 191)
			aproxDBM = -25;
		else if (data.rssi >= 199)
			aproxDBM = -200;
		
		data.rssiDBM = aproxDBM;
		int rssiPerc = aproxDBM*1.0989+127.4725;
		data.rssiPercentage=rssiPerc;
    }    
}

void GpsDriver::GetPosition(){
	regex error_regex("\\+CME ERROR:", regex_constants::ECMAScript | regex_constants::icase);
	regex success_regex("\\+QGPSLOC:", regex_constants::ECMAScript | regex_constants::icase);

	string reply = SendATCommand("AT+QGPSLOC?\r");
	try
	{
		vector<string> reply_split = split(reply, '\n'); 
		for(int i=0;i<reply_split.size();i++)
		{
			if (regex_search(reply_split[i], error_regex)) {
				vector<string> split_1 = split(reply_split[i],' ');
				string str = split_1.at(2);
				string::iterator end_pos = remove(str.begin(), str.end(), ' ');
				str.erase(end_pos, str.end());
				int err = stoi(str);
				if (err == 505)
				{
					dgnst.session = true;
				}
				else if (err == 516)
				{
					dgnst.fixed = false;
				}
			} else if (regex_search(reply_split[i], success_regex)) {
				vector<string> split_1 = split(reply_split[i],' ');
				vector<string> split_2 = split(split_1[1],',');
				GetUTCTime(split_2[0]);
				GetLatitud(split_2[1]); 
				GetLongitud(split_2[2]);
				GetAccuracy(split_2[3]);
				GetAltitud(split_2[4]);
				GetUTCDate(split_2[9]);
				data.satellites = GetSatellites(split_2[10]);
				
				if(data.latValid == true and data.longValid == true)
				{
					if(data.latitud != 0 and data.longitud != 0)
					{
						data.gpsValid = true;
						dgnst.fixed = true;
					}
					else
					{
						data.gpsValid = false;
						dgnst.fixed = false;
					}
				}
				else
				{
					data.gpsValid = false;
					dgnst.fixed = false;
				}
			}
		}
	}catch(...){
		ROS_WARN_STREAM("getPosicion() error. Not fatal. Just ignore it.");
    }
}

void GpsDriver::GetUTCTime(string utc){  
    if(utc.size() > 0)
    {
		vector<string> split_utc = split(utc,'.');
		data.UTCTime = split_utc.at(0);
    }
    else
    {
		data.UTCTime = "0";
    }
}

void GpsDriver::GetLatitud(string latitude){
    if(latitude.size() > 0)
    {
		double decimal;
		data.latHemisphere = latitude.back();    
		vector<string> split_2 = split(latitude,'.');
		string grados_str = split_2.at(0).substr(0,split_2.at(0).size()-2);
		string minutos_str = split_2.at(0).substr(split_2.at(0).size()-2,split_2.at(0).size());
		string segundos_str = "." + split_2.at(1).substr(0,split_2.at(1).size()-1);
		double grados = atof(grados_str.c_str());
		double minutos = atof(minutos_str.c_str());
		double segundos = atof(segundos_str.c_str());
		decimal = grados + minutos/60 + segundos*60/3600;
		if (data.latHemisphere == "S") decimal *= -1;
		data.latitud = decimal;
		data.latValid = true;
    }
    else
    {
		data.latValid = false;
    }
}

void GpsDriver::GetLongitud(string longitud){
    if(longitud.size() > 0)
    {
		double decimal;
		char long_dir = longitud.back();
		data.lonHemisphere = longitud.back();
		vector<string> split_2 = split(longitud,'.');
		string grados_str = split_2.at(0).substr(0,split_2.at(0).size()-2);
		string minutos_str = split_2.at(0).substr(split_2.at(0).size()-2,split_2.at(0).size());
		string segundos_str = "." + split_2.at(1).substr(0,split_2.at(1).size()-1);
		double grados = atof(grados_str.c_str());
		double minutos = atof(minutos_str.c_str());
		double segundos = atof(segundos_str.c_str());
		decimal = grados + minutos/60 + segundos*60/3600;
		if (data.lonHemisphere == "W") decimal *= -1;
		data.longitud = decimal;
		data.longValid = true;
    }
    else
    {
		data.longValid = false;
    }
}

void GpsDriver::GetAccuracy(string hdop){
    data.accuracy = stod(hdop);
}

void GpsDriver::GetAltitud(string altitud){	
    if(altitud.size() > 0)
    {
		const char *alt = altitud.c_str();
		data.altitud = atof(alt);
    }
}

void GpsDriver::GetUTCDate(string date){ 
    data.UTCDate = date;
}

int GpsDriver::GetSatellites(string satellites){
    try
    {
		if(satellites.size() > 0)
			return stoi(satellites);
		else
			return 0;
    }
    catch(...)
    {
		return 0;
    }	
}

void GpsDriver::GetNetworkInformation(){
	string reply = SendATCommand("AT+QNWINFO\r");
	// ROS_DEBUG_STREAM(reply);
	try
	{
		vector<string> reply_split = split(reply, '\n');
		for(int i=0;i<reply_split.size();i++)
		{
			if(reply_split[i].size() > 20)
			{
				vector<string> split_1 = split(reply_split[i],':');
				vector<string> split_2 = split(split_1[1],',');
				// data.provider = split_2.at(1).c_str();
				string provider_full = split_2.at(1).c_str();
				string provider = provider_full.substr(1,6);
				if (provider == "310410"){
					data.provider = "AT&T";
				}
				else if (provider == "310260"){
					data.provider = "T-Mobile";
				}
				else {
					data.provider = "";
				}
				data.tech = split_2.at(0).c_str();
				data.band = split_2.at(2).c_str();
			}
		}
	} catch(...){
		ROS_WARN_STREAM("GetNetworkInformation() error. Not fatal. Just ignore it.");
    }

}

GpsDriver::~GpsDriver(){
    DeinitializeModule();
}