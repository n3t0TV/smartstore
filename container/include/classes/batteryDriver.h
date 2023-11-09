#include <ros/master.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <unistd.h>
#include <wiringPiI2C.h>

#include <std_msgs/Int32.h>

#include <stdint.h>

#define ADC_ADDRESS 0x48
// Single ended measurement, channel 3, internal reference on, adc on
#define CONFIG_BYTE 0b11010100
#define RET_ERR -1

// Voltage divider resistor values
#define R1 10000.0
#define R2 3000.0
// ADC positive reference voltage
#define VREF_PLUS 3.0
// ADC max counts
#define ADC_MAX 255.0

// Critical minimum battery level (%)
#define MINBATT 2

// Bitshift for oversampling filter
#define FILTER_BITSHIFT 5

using namespace std;
using namespace ros;

class BatteryDriver {
 public:
  BatteryDriver(NodeHandle);
  ~BatteryDriver();
  void BatteryLevelCallback();

 private:
  NodeHandle nh;
  Publisher batteryLevelPub;

  int log_level, fd;
  string node_name;
  bool low_batt_shutdown;
  std_msgs::Int32 batteryLevel;

  void BatteryWatcher(const int battery_level);
};

BatteryDriver::BatteryDriver(NodeHandle nh_priv) {
  node_name = this_node::getName();
  nh_priv.param("log_level", log_level, 0);
  nh_priv.param("low_batt_shutdown", low_batt_shutdown, true);
  console::levels::Level console_level = (console::levels::Level)log_level;
  if (console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level)) {
    console::notifyLoggerLevelsChanged();
  }

  batteryLevelPub = nh.advertise<std_msgs::Int32>("/sensor_topic/battery", 1);

  fd = wiringPiI2CSetup(ADC_ADDRESS);
  if (fd == RET_ERR) {
    ROS_ERROR_STREAM("Failed to init I2C communication.");
  }
  int ret = wiringPiI2CWrite(fd, CONFIG_BYTE);
  if (ret == RET_ERR) {
    ROS_ERROR_STREAM(node_name << " --- I2C: failed to write to ADC address (0x"
                               << std::hex << ADC_ADDRESS << ")");
  }

  /* Initialize the battery msg to detect changes */
  batteryLevel.data = -1;
}

// Lookup table for voltage levels
static const double voltages_table[] = {
9.448, 9.511, 9.573, 9.636, 9.698, 9.761, 9.823, 9.886, 9.948, 10.011, 10.074, 10.136, 10.199, 10.261, 10.324,
10.386, 10.449, 10.511, 10.574, 10.636, 10.699, 10.761, 10.824, 10.886, 10.949, 11.011, 11.074, 11.136, 11.199,
11.261, 11.324, 11.386, 11.449, 11.511, 11.574, 11.637, 11.699, 11.762, 11.824, 11.887, 11.949, 12.012, 12.074,
12.137, 12.199, 12.262, 12.324, 12.387, 12.449, 12.512
};

// Lookup table for battery percentage levels
static const double percentages_table[] = {
0.000, 0.646, 1.684, 2.644, 3.411, 3.563, 5.715, 7.209, 8.942, 10.430, 12.818, 14.706, 18.142, 20.417, 23.452,
26.971, 30.645, 32.756, 36.489, 39.647, 41.055, 44.407, 46.817, 49.572, 51.343, 53.414, 55.591, 57.725, 60.100,
63.068, 66.002, 68.613, 71.394, 73.433, 75.392, 77.539, 79.667, 82.359, 85.055, 90.552, 92.970, 96.094, 97.865,
99.144, 99.751, 99.969, 99.998, 99.999, 99.999, 100.000
};

size_t lookup_tables_size = sizeof(voltages_table)/sizeof(double);

// Voltage levels for a full-charged battery or an empty one
#define	VMAX	voltages_table[lookup_tables_size-1]
#define	VMIN	voltages_table[0]

/**
 * @brief
 *
 * @param x
 * @param x0
 * @param x1
 * @param y0
 * @param y1
 * @return double
 */
double Interpolation(double x, double x0, double x1, double y0, double y1) {
  return (x - x0) * (y1 - y0) / (x1 - x0) + y0;
}

/**
 * @brief
 *
 * @param vbat
 * @return int
 */
int Voltage2Percent(double vbat) {
  if (vbat < VMIN)
    vbat = VMIN;
  else if (vbat > VMAX)
    vbat = VMAX;

  int index;
  int len = sizeof(voltages_table) / sizeof(double);
  for (int i = 0; i != len; i++) {
    if (vbat <= voltages_table[i]) {
      index = i;
      break;
    }
  }
  // Lookup table values for interpolation
  double v1 = voltages_table[index - 1];
  double v2 = voltages_table[index];
  double percentage1 = percentages_table[index - 1];
  double percentage2 = percentages_table[index];

  return (int)Interpolation(vbat, v1, v2, percentage1, percentage2);
}

/**
 * @brief
 *
 * @param adc_read
 * @return double
 */
double ADC2Voltage(int adc_read) {
  return ((((double)adc_read / ADC_MAX)) * VREF_PLUS * (R2 + R1)) / R2;
}

void BatteryDriver::BatteryLevelCallback() {
  int ret = wiringPiI2CWrite(fd, CONFIG_BYTE);
  if (ret == RET_ERR) {
    ROS_WARN_STREAM_THROTTLE(
        60, node_name << " --- I2C: failed to write to ADC address (0x"
                      << std::hex << ADC_ADDRESS << ")");
    return;
  }


  uint32_t adc_read = 0;
	uint16_t iterations = (1<<FILTER_BITSHIFT);
	for(uint16_t i=0; i != iterations; i++){
		adc_read += (uint32_t)(wiringPiI2CRead(fd));
	}
	adc_read >>= FILTER_BITSHIFT;


  double voltage = ADC2Voltage((int)adc_read);
  int current_battery = Voltage2Percent(voltage);
  if (current_battery != batteryLevel.data) {
    batteryLevel.data = current_battery;
    batteryLevelPub.publish(batteryLevel);
    BatteryWatcher(current_battery);
  }
}

void BatteryDriver::BatteryWatcher(const int battery_level) {
  /* If low battery shutdown is not enabled return */
  if (!low_batt_shutdown) {
    return;
  }

  if (battery_level <= MINBATT) {
    ROS_WARN_STREAM(
        node_name
        << "--- Stopping mss & poweroff (minimum battery level detected)");
    int nRet = system(
        "sudo shutdown -P +1 'The vehicle is about to shutdown (minimum battery "
        "level detected)'");
    if (nRet == 0)
      nRet = system("systemctl --user stop mss_ros_launcher.service");
  }
}

BatteryDriver::~BatteryDriver() {}
