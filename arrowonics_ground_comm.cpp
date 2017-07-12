#include <px4.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_defines.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <math.h>
#include <stdint.h>
#include <sys/select.h>
#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>	//AccZ(vibration) and angle oscillation


#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <limits.h>
#include <nuttx/serial/serial.h>
#include <time.h>


//#define MAG_BUFFER_LENGTH 25

//#define DEBUG_PRINTF //comment this line to stop printf

extern "C" __EXPORT int ground_comm_main(int argc, char *argv[]);
static void usage(void);

class GroundComm {
public:
    /**
     * Constructor
     */
    GroundComm();

    /**
     * Destructor, also kills task.
     */
    ~GroundComm();

    /**
     * Start task.
     *
     * @return      OK on success.
     */
    int start();

private:

    bool _task_should_exit; /**< if true, task should exit */
	int _control_task; /**< task handle for task */

	//System Variables
	int uart;

	//Topic Subscription
	int sensor_combined_sub_fd;

	struct sensor_combined_s sensor;

	//GCS - Leader Communication Network Message Lengths
	int MSGLENGTH_COMMAND; //should be 10
	
    //Shim for calling task_main from task_create.
    static void task_main_trampoline(int argc, char *argv[]);

    //Main task.
    void task_main();
	
    //Check for changes in subscribed topics.
	void poll_subscriptions();

	//Set up UART port for receiving messages
	bool setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control);

	//Checksum helper function
	uint16_t crc16_update(uint16_t crc, char a);

	//Checksum calculation
	uint16_t calcCRC16(unsigned char *serno, int length);

	//Checksum calculation
	uint16_t checksum(unsigned char *data, int length);

	//Initialize UART
	void uart_init();

	//Public additional commands from GCS
	void publish_additional_commanded_modes(int aircraftID, int command_int);

	//Create 4 byte array from integer
	void getFourByteArray(int num, unsigned char *array);

	//Copy array into another
	void copyArray(unsigned char *dest, int destIndex, unsigned char *source, int sourceLength);

	//Send all vehicle status in the group
	void send_all_position(int groupID, int ownID);

	//Compute vehicle status using 3 parameters: gps status, arm status, and flight mode
	int vehicleStatus(bool gps, bool armed, bool automode);

	void send_group_pos_message(int groupID, int acID, double lat, double lon, double alt, int status, int fix, int nSat, float eph, float battV, int alert);

	//Create position packet for individual vehicle
	void createVehiclePosPacket(int msgID, int grpID, int acID, double lat, double lon, double alt, int status, int fix, int nSat, float eph, float battV, int alert, unsigned char *buffer);
};

namespace ground_comm {

GroundComm *ground_comm;

}

GroundComm::GroundComm() :
		_task_should_exit(false),
		_control_task(-1),
		uart(-1),
		formationMode_cmd(-1),
		armed_cmd(false),
        compass_calibration_completed(false),
        compass_calibration_requested(false),
        commanded_modes_pub_fd(nullptr),
        polling_request_pub_fd(nullptr),
        formation_home_pub_fd(nullptr),
        gsc_commands_pub_fd(nullptr),
        commanded_arm_pub_fd(nullptr),
        sensor_calibration_command_pub_fd(nullptr),
        no_fly_zone_parameters_pub_fd(nullptr),
        return_to_home_parameters_pub_fd(nullptr),
		vehicle_global_position_sub_fd(-1),
		vehicle_control_mode_sub_fd(-1),
		mesh_network_sub_fd(-1),
		vehicle_local_position_sub_fd(-1),
		vehicle_gps_position_sub_fd(-1),
		battery_status_sub_fd(-1),
        sensor_combined_sub_fd(-1),
		formation_cmdr_error_sub_fd(-1),
        arrow_time_sub_fd(-1),
        sensor_calibration_status_sub_fd(-1),
		GROUP_ID(0),
		AC_ID(0),
		LEADER(0),
		MSGLENGTH_COMMAND(10),
		systemAlert(0),
		max_accZ(-98),
		min_accZ(-98),
		gpsStatus(false),
		autoFormMode(false),
        MESHSIZE(12),
        gsc_led_cmd_counter(0),
        gsc_led_cmd_flag(false),
        mag_sum(0.0f),
        mag_buffer_head(0)
{
	memset(&sensor, 0, sizeof(sensor));
}

GroundComm::~GroundComm()
{
    if (_control_task != -1) {
        /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;

        /* wait for a second for the task to quit at our request */
        unsigned i = 0;

        do {
            /* wait 20ms */
            usleep(20000);

            /* if we have given up, kill it */
            if (++i > 50) {
                task_delete(_control_task);
                break;
            }
        } while (_control_task != -1);
    }

    ground_comm::ground_comm = nullptr;
}

static void usage()
{
	fprintf(stderr,
			"usage: ground_comm start [-d <devicename>]\n"
			"       ground_comm stop\n"
			"       ground_comm status\n");
	exit(1);
}

int ground_comm_main(int argc, char *argv[]) {
    if (argc < 1) {
        errx(1, "usage: ground_comm {start|stop|status}");
    }

    if (!strcmp(argv[1], "start")) {

        if (ground_comm::ground_comm != nullptr) {
            errx(1, "already running");
        }

        ground_comm::ground_comm = new GroundComm;

        if (ground_comm::ground_comm == nullptr) {
            errx(1, "alloc failed");
        }

        if (OK != ground_comm::ground_comm->start()) {
            delete ground_comm::ground_comm;
            ground_comm::ground_comm = nullptr;
            err(1, "start failed");
        }

        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        if (ground_comm::ground_comm == nullptr) {
            errx(1, "not running");
        }

        delete ground_comm::ground_comm;
        ground_comm::ground_comm = nullptr;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (ground_comm::ground_comm) {
            errx(0, "running");

        } else {
            errx(1, "not running");
        }
    }

    warnx("unrecognized command");

    usage();
    return 1;
}

int GroundComm::start() {
    ASSERT(_control_task == -1);

    /* start the task */
    _control_task = px4_task_spawn_cmd("[ATL] ground_comm",     //name
            SCHED_DEFAULT,                          //priority
            SCHED_PRIORITY_DEFAULT,                 //scheduler
            1500,                                   //stack size
            (main_t) &GroundComm::task_main_trampoline, nullptr);

    if (_control_task < 0) {
        warn("task start failed");
        return -errno;
    }

    return OK;
}

void GroundComm::task_main_trampoline(int argc, char *argv[])
{
    ground_comm::ground_comm->task_main();
}


void GroundComm::task_main()
{
	warnx("[Ground Comm] starting\n");



	//Subscribing Topics
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));

	//Initialize UART Port
	uart_init();

	while (!_task_should_exit) {
		poll_subscriptions();
		// TODO Add send message, and populate message

		usleep(30000); //30ms
	}

    _control_task = -1;
    _exit(0);
}

bool GroundComm::setup_port(int fd, int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
	//struct termios options;
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		//fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return false;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
	                    INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);
	config.c_oflag = 0;
	config.c_lflag = 0;
	config.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	config.c_cflag |= CS8;
	config.c_cflag &= ~CRTSCTS;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 5; // was 0
	switch (baud)
	{
		case 1200:
			if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 1800:
			cfsetispeed(&config, B1800);
			cfsetospeed(&config, B1800);
			break;
		case 9600:
			cfsetispeed(&config, B9600);
			cfsetospeed(&config, B9600);
			break;
		case 19200:
			cfsetispeed(&config, B19200);
			cfsetospeed(&config, B19200);
			break;
		case 38400:
			if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 57600:
			if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		case 115200:
			if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
			{
				//fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
				return false;
			}
			break;
		default:
			//fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
			return false;

			break;
	}
	if(tcsetattr(fd, TCSANOW, &config) < 0)
	{
		//fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
		return false;
	}
	return true;
}

uint16_t GroundComm::crc16_update(uint16_t crc, char a)
{
	int i;
	crc ^= a;
	for (i = 0; i < 8; ++i)
	{
		if ((crc & 1) != 0)
			crc = (crc >> 1) ^ 0xA001;
		else
			crc = (crc >> 1);
	}
	return crc;
}

uint16_t GroundComm::calcCRC16(unsigned char *serno, int length)
{
		uint16_t crc = 0;
	uint16_t i;

	unsigned char temp[31] = {0};
	memcpy(temp, &serno[0], length);

	for (i = 0; i < 31; i++)
		crc = crc16_update(crc, temp[i]);

	return crc; // must be 0
}

uint16_t GroundComm::checksum(unsigned char *data, int length)
{
	uint16_t sum = calcCRC16(data, length);
	return sum;
}

void GroundComm::uart_init()
{
	uart  = open("/dev/ttyS1", O_RDWR | O_NOCTTY);		// ttyS1 = telem1 port on pixhawk
	setup_port(uart,57600,8,1,false,false); 			//baudrate = 57600
}

void GroundComm::publish_additional_commanded_modes(int aircraftID, int command_int)
{
	gsc.acid = aircraftID;
	gsc.cmd = command_int;
    if (command_int == 3)
        gsc.cmd = 6;                //6 is the sensor calibration for the topic, but 3 is sensor calibration for the msg
	#ifdef DEBUG_PRINTF
		printf("Additional Commands - acid: %d\t cmd: %d\n", gsc.acid, gsc.cmd);
    #endif
    if (gsc_commands_pub_fd == nullptr)
        gsc_commands_pub_fd = orb_advertise(ORB_ID(gsc_commands), &gsc);
    else
        orb_publish(ORB_ID(gsc_commands), gsc_commands_pub_fd, &gsc);

	if (command_int == 0 || command_int == 1)
	{
		gsc_led_cmd_flag = true;	//start timer for reset gsc command
		gsc_led_cmd_counter = 250;	//reset gsc command counter for 5s
	}
}

void GroundComm::poll_subscriptions()
{
	bool updated;

	orb_check(sensor_combined_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor);
	}
}


/*Converts integer to 4 Byte character array*/
void GroundComm::getFourByteArray(int num, unsigned char *array)
{
	//Masks out each Byte of number and assigns to character array
	array[3] = (num >> 24) & 0xFF;
	array[2] = (num >> 16) & 0xFF;
	array[1] = (num >> 8) & 0xFF;
	array[0] = num & 0xFF;
}

/*Copies array source into array dest, beginning at location destIndex
 * destIndex+sourceLength must be less than length of dest or else out of bounds
 */
void GroundComm::copyArray(unsigned char *dest, int destIndex, unsigned char *source, int sourceLength)
{
	int i;
	for (i = 0; i < sourceLength; i++)
	{
		dest[destIndex+i] = source[i];
	}
}



void GroundComm::send_group_pos_message(int groupID, int acID, double lat, double lon, double alt, int status, int fix, int nSat, float eph, float battV, int alert)
{
    unsigned char buffer[MSGLENGTH_POSITION];
	//Set message
	int messageID = 7;

	//Create packet to be sent
	createVehiclePosPacket(messageID, groupID, acID, lat, lon, alt, status, fix, nSat, eph, battV, alert, buffer);

	//Write packet to Xbee through uart
    write(uart, buffer, MSGLENGTH_POSITION);
	//usleep(1000);
	//write(uart, buffer, 22);
}

void GroundComm::createVehiclePosPacket(int msgID, int grpID, int acID, double lat, double lon, double alt, int status, int fix, int nSat, float eph, float battV, int alert, unsigned char *buffer)
{
	//Packet header, not using for now
	char header[] = "X";
	//Message ID
	char msg_char = (char)msgID;
	char status_char = (char)status;
	char nsat_char = (char)nSat;
	char alert_char = (char)alert;
	unsigned int ephInt = eph*10;
	if (ephInt > 255)		//limit upper bound to 1 byte
		ephInt = 255;
	unsigned int battVInt = battV*10;
	char eph_char = (char)ephInt;
	char battV_char = (char)battVInt;
	char fix_char = (char)fix;
	//Arrays to convert to characters
	unsigned char groupID_char[3], acID_char[3], lat_char[4], lon_char[4], alt_char[4], checkSum_char[2];

	//Convert to char array
	getThreeByteArray(grpID, groupID_char);
	getThreeByteArray(acID, acID_char);
	getFourByteArray(lat*1000000, lat_char);
	getFourByteArray(lon*1000000, lon_char);
	getFourByteArray(alt*100, alt_char);

	//Fill buffer
	buffer[0] = header[0];
	buffer[1] = msg_char;
	copyArray(buffer, 2, groupID_char, 1);
	copyArray(buffer, 3, acID_char, 1);
	copyArray(buffer, 4, lat_char, 4);
	copyArray(buffer, 8, lon_char, 4);
	copyArray(buffer, 12, alt_char, 2);
	buffer[14] = status_char;
	buffer[15] = fix_char;
	buffer[16] = nsat_char;
	buffer[17] = eph_char;
	buffer[18] = battV_char;
	buffer[19] = alert_char;

//		printf("Data: ");
//		int k;
//		for (k=2;k<25;k++)
//			printf("%d  ",buffer[k]);
//		printf("\n");
//		printf("Message: %c\t %c\t %c\t %c\n", nsat_char, eph_char, battV_char, alert_char);
	//Calculate checksum
	unsigned char toCheck[18];
	memcpy(toCheck, &buffer[2],18);

	int checkSum = checksum(toCheck, 18);
	getThreeByteArray(checkSum, checkSum_char);
	copyArray(buffer, 20, checkSum_char, 2);
}
