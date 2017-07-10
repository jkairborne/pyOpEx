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
#include <uORB/topics/commanded_modes.h>	//flight mode command from gcs
#include <uORB/topics/gsc_commands.h>	//additional gcs commands
#include <uORB/topics/commanded_arm.h>	//arming mode command from gcs
#include <uORB/topics/vehicle_global_position.h>	//Own position (lat, lon)
#include <uORB/topics/mesh_network.h>	//Group position
#include <uORB/topics/vehicle_control_mode.h>	//Own vehicle control mode
#include <uORB/topics/vehicle_local_position.h>		//Own position (altitude)
#include <uORB/topics/vehicle_gps_position.h>		//Own GPS Fix Type
#include <uORB/topics/polling_request.h>	//polling request from GCS
#include <uORB/topics/battery_status.h>		//battery voltage
#include <uORB/topics/sensor_combined.h>	//AccZ(vibration) and angle oscillation
#include <uORB/topics/formation_home.h>		//home position set or not
#include <uORB/topics/formation_cmdr_error.h>	//checks for wrong missionID
#include <uORB/topics/arrow_time.h>	//mission time
#include <uORB/topics/sensor_calibration_command.h>    //calibrate sensors
#include <uORB/topics/sensor_calibration_status.h>      //compass calibration completeness
#include <uORB/topics/no_fly_zone_parameters.h>     //for collision free emergency landing
#include <uORB/topics/return_to_home_parameters.h>      //for return to home emergency landing

#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <limits.h>
#include <nuttx/serial/serial.h>
#include <time.h>

#define NEVER_SEND_RAW_GPS

#define MAG_BUFFER_LENGTH 25

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
	int formationMode_cmd;
	bool armed_cmd;
    bool compass_calibration_completed;
    bool compass_calibration_requested;
	FILE *f;

	//Topic Publication
	struct commanded_modes_s commandModes;
	struct polling_request_s polling;
	struct formation_home_s home;
	struct gsc_commands_s gsc;
	struct commanded_arm_s commandArm;
    struct sensor_calibration_command_s sensorCalib;
    struct no_fly_zone_parameters_s noFlyZone;
    struct return_to_home_parameters_s returnHomeParam;
	orb_advert_t commanded_modes_pub_fd;
	orb_advert_t polling_request_pub_fd;
	orb_advert_t formation_home_pub_fd;
	orb_advert_t gsc_commands_pub_fd;
	orb_advert_t commanded_arm_pub_fd;
    orb_advert_t sensor_calibration_command_pub_fd;
    orb_advert_t no_fly_zone_parameters_pub_fd;
    orb_advert_t return_to_home_parameters_pub_fd;

	//Topic Subscription
	int vehicle_global_position_sub_fd;
	int vehicle_control_mode_sub_fd;
	int mesh_network_sub_fd;
	int vehicle_local_position_sub_fd;
	int vehicle_gps_position_sub_fd;
	int battery_status_sub_fd;
	int sensor_combined_sub_fd;
	int formation_cmdr_error_sub_fd;
    int arrow_time_sub_fd;
    int sensor_calibration_status_sub_fd;
	struct vehicle_global_position_s global_pos;
	struct vehicle_control_mode_s ctl_mode;
	struct mesh_network_s meshNetwork;
	struct vehicle_local_position_s local_pos;
	struct vehicle_gps_position_s gps_pos;
	struct battery_status_s battery;
	struct sensor_combined_s sensor;
	struct formation_cmdr_error_s cmd_error;
    struct arrow_time_s arrowTime;
    struct sensor_calibration_status_s calibStatus;

	int GROUP_ID;
	int AC_ID;
	int LEADER;

	//GCS - Leader Communication Network Message Lengths
	int MSGLENGTH_COMMAND;
	int MSGLENGTH_HEARTBEAT;
	int MSGLENGTH_HOME;
	int MSGLENGTH_GCSCMD;
    int MSGLENGTH_POSITION;
    int MSGLENGTH_FANCYLANDING;

	//System Alert
	int systemAlert;
	int max_accZ;
	int min_accZ;
	bool gpsStatus;
	bool autoFormMode;

	//Mesh Network
	int MESHSIZE;

	//Identify LED timer
	int gsc_led_cmd_counter;	//5 seconds at 20000us interval
    bool gsc_led_cmd_flag;

    //total magnetic field strength
    float total_mag_field_strength[MAG_BUFFER_LENGTH];
    float mag_sum;
    int mag_buffer_head;

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

	//Convert byte array to int
	int getIntFromArray(unsigned char *array, int begin, int length);

	//Initialize UART
	void uart_init();

	//Main read byte function
	void ground_read(char filename[]);

	//Decode home position message from GCS
	void decode_home_message(unsigned char *buffer);

	//Decode heartbeat message from GCS
	void decode_heartbeat_message(unsigned char *buffer);

	//Decode arming and flight mode command message from GCS
	int decode_command_message(unsigned char *buffer);

	//Decode additional command messages from GCS
	void decode_additional_cmd_message(unsigned char *buffer);

    int decode_fancy_landing_message(unsigned char *buffer);

	//Public additional commands from GCS
	void publish_additional_commanded_modes(int aircraftID, int command_int);

	//Create 3 byte array from integer
	void getThreeByteArray(int num, unsigned char *array);

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

	//Check for system alerts in vibration
	int checkAlerts(void);

	//Keep track of accZ for vibration detection
	void updateAccZ(float accZ_curr);

	//Logging commands from GCS
	void log_command(int command, char filename[]);
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
		MSGLENGTH_COMMAND(11),
        MSGLENGTH_HEARTBEAT(8),
		MSGLENGTH_HOME(22),
        MSGLENGTH_GCSCMD(11),
        MSGLENGTH_POSITION(22),
        MSGLENGTH_FANCYLANDING(16),
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
	memset(&commandModes, 0, sizeof(commandModes));
	memset(&polling, 0, sizeof(polling));
	memset(&home, 0, sizeof(home));
	memset(&gsc, 0, sizeof(gsc));
	memset(&commandArm, 0, sizeof(commandArm));
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&ctl_mode, 0, sizeof(ctl_mode));
	memset(&meshNetwork, 0, sizeof(meshNetwork));
	memset(&local_pos, 0, sizeof(local_pos));
	memset(&gps_pos, 0, sizeof(gps_pos));
	memset(&battery, 0, sizeof(battery));
	memset(&sensor, 0, sizeof(sensor));
	memset(&cmd_error, 0, sizeof(cmd_error));
    memset(&arrowTime, 0, sizeof(arrowTime));
    memset(&sensorCalib, 0, sizeof(sensorCalib));
    memset(&noFlyZone, 0, sizeof(noFlyZone));
    memset(&returnHomeParam, 0, sizeof(returnHomeParam));

    for(int i=0;i<MAG_BUFFER_LENGTH;i++)
    {
        total_mag_field_strength[i]=0.0f;
    }
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

	param_t leader_ptr = param_find("SYS_LEADER");
	param_get(leader_ptr, &LEADER);

	param_t groupID_ptr = param_find("SYS_GROUPID");
	param_get(groupID_ptr, &GROUP_ID);

    param_t acID_ptr = param_find("SYS_ACID");
    param_get(acID_ptr, &AC_ID);

	if (!LEADER)
	{
		warnx("Not leader, exiting...\n");
		_control_task = -1;
		_exit(0);
	}

	//Subscribing Topics
	vehicle_global_position_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	vehicle_control_mode_sub_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
	mesh_network_sub_fd = orb_subscribe(ORB_ID(mesh_network));
	vehicle_local_position_sub_fd = orb_subscribe(ORB_ID(vehicle_local_position));
	vehicle_gps_position_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
	battery_status_sub_fd = orb_subscribe(ORB_ID(battery_status));
	sensor_combined_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	formation_cmdr_error_sub_fd = orb_subscribe(ORB_ID(formation_cmdr_error));
    arrow_time_sub_fd = orb_subscribe(ORB_ID(arrow_time));
    sensor_calibration_status_sub_fd = orb_subscribe(ORB_ID(sensor_calibration_status));

	//Initialize logging to file
    char filename[]="/fs/microsd/comm_log.txt";
	f = fopen(filename, "w+");
	fclose(f);

	//Initialize UART Port
	uart_init();

	//Set up for read timeout
	int rv;
	fd_set set;
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 10000;

	while (!_task_should_exit) {
		poll_subscriptions();
		updateAccZ(sensor.accelerometer_m_s2[2]);
		FD_ZERO(&set);
		FD_SET(uart, &set);
		rv = select(uart + 1, &set, NULL, NULL, &timeout);
		if (rv > 0)		//only read when there's data
			ground_read(filename);
		if (gsc_led_cmd_flag)
		{
			if (gsc_led_cmd_counter == 0)	//After 5s, reset flag and counter, and reset gsc command
			{
				gsc_led_cmd_flag = false;
				gsc.acid = 0;
				gsc.cmd = 0;
				orb_publish(ORB_ID(gsc_commands), gsc_commands_pub_fd, &gsc);
			}
			else
			{
				gsc_led_cmd_counter--;
			}
		}
		usleep(10000);
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

int GroundComm::getIntFromArray(unsigned char *array, int begin, int length)
{
	int i, num = 0;
	for (i = 0; i < length; i++)
	{
        num += (array[i+begin] << (i * 8));
	}
	return num;
}

void GroundComm::uart_init()
{
	uart  = open("/dev/ttyS1", O_RDWR | O_NOCTTY);		// ttyS1 = telem1 port on pixhawk
	setup_port(uart,57600,8,1,false,false); 			//baudrate = 57600
}

void GroundComm::ground_read(char filename[])
{
	char msg[1];
	int n = read(uart, &msg, 1);
	if (n>0 && msg[0] == 88) //X header
	{
		//Check message
		int m = read(uart, &msg,1); //Get second byte
		if (m>0)
		{
			if (msg[0] == 3)	//Heartbeat message from GSC
			{
				#ifdef DEBUG_PRINTF
					//printf("Heartbeat Msg\n");
                #endif
				int bufferOffset;
                unsigned char buffer[MSGLENGTH_HEARTBEAT-2];
				for(bufferOffset = 0; bufferOffset < MSGLENGTH_HEARTBEAT - 2; bufferOffset++)
				{
					read(uart, &msg, 1);
                    buffer[bufferOffset] = msg[0];
				}
                decode_heartbeat_message(buffer);
                int delay = GROUP_ID * 50000;	//50ms * acID
				usleep(delay);
				send_all_position(GROUP_ID, AC_ID);
                //log_command(0, filename);
			}
			else if (msg[0] == 8)		//Command message from GSC
			{
				#ifdef DEBUG_PRINTF
					printf("Command Msg\n");
                #endif
				int bufferOffset, command_log;
				unsigned char buffer[MSGLENGTH_COMMAND-2];
				for(bufferOffset = 0; bufferOffset < MSGLENGTH_COMMAND - 2; bufferOffset++)
				{
					read(uart, &msg,1);
					buffer[bufferOffset] = msg[0];
				}
				command_log = decode_command_message(buffer);
				log_command(command_log, filename);
			}

			else if (msg[0] == 9)
			{
				#ifdef DEBUG_PRINTF
					printf("Home Msg\n");
                #endif
				int bufferOffset;
				unsigned char buffer[MSGLENGTH_HOME-2];
				for(bufferOffset = 0; bufferOffset < MSGLENGTH_HOME - 2; bufferOffset++)
				{
					read(uart, &msg,1);
					buffer[bufferOffset] = msg[0];
				}
				decode_home_message(buffer);
				log_command(7, filename);
			}
			else if (msg[0] == 10)
			{
				#ifdef DEBUG_PRINTF
					printf("GSC CMD\n");
                #endif
				int bufferOffset;
				unsigned char buffer[MSGLENGTH_GCSCMD-2];
				for(bufferOffset = 0; bufferOffset < MSGLENGTH_GCSCMD - 2; bufferOffset++)
				{
					read(uart, &msg,1);
					buffer[bufferOffset] = msg[0];
				}
				decode_additional_cmd_message(buffer);
				log_command(6, filename);
			}
            else if (msg[0] == 13)
            {
                #ifdef DEBUG_PRINTF
                    printf("Fancy Landing CMD\n");
                #endif
                int bufferOffset;
                unsigned char buffer[MSGLENGTH_FANCYLANDING-2];
                for(bufferOffset = 0; bufferOffset < MSGLENGTH_FANCYLANDING - 2; bufferOffset++)
                {
                    read(uart, &msg,1);
                    buffer[bufferOffset] = msg[0];
                }
                int landing_type = decode_fancy_landing_message(buffer);
                log_command(landing_type, filename);
            }
		}
	}
}

void GroundComm::log_command(int command, char filename[])	//command: 0 = heartbeat, 1 = arm, 2 = start, 3 = stop, 4 = disarm, 5 = land, 6 = flash LED, 7 = home, 8 = standard landing, 9 = collision-free landing, 10 - contract, 11 - resume
{
    float curr_time = arrowTime.time_ms/1000.0f;
    if (command >= 0 && command < 12)
	{
        f=fopen(filename, "a");
        if(f!=NULL)
        {
            if (command == 0)
            {
            }
            else if (command == 1)
            {
                char str[] = "Arm";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 2)
            {
                char str[] = "Start";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 3)
            {
                char str[] = "Stop";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 4)
            {
                char str[] = "Disarm";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 5)
            {
                char str[] = "Land";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 6)
            {
                char str[] = "Flash LED";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 7)
            {
                char str[] = "Home Position / Mission ID:";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s %.7f, %.7f, %d\r\n", str, home.lat, home.lon, home.mission_id);
            }
            else if (command == 8)
            {
                char str[] = "Return to Home Landing:";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s %d\r\n", str, (int)returnHomeParam.z);
            }
            else if (command == 9)
            {
                char str[] = "Collision Free Landing:";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s %d, %d, %.7f, %.7f, %d\r\n", str, noFlyZone.grpid, noFlyZone.acid, (double)noFlyZone.x, (double)noFlyZone.y, (int)noFlyZone.z);
            }
            else if (command == 10)
            {
                char str[] = "Contract";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            else if (command == 11)
            {
                char str[] = "Resume";
                fprintf(f, "%.2f\t", (double)curr_time);
                fprintf(f, "%s\r\n", str);
            }
            fflush(f);
            fclose(f);
        }
	}
}

void GroundComm::decode_home_message(unsigned char *buffer)
{
	unsigned char toCheckRx[MSGLENGTH_HOME - 4];
	memcpy(toCheckRx, &buffer[0], MSGLENGTH_HOME - 4);
	int checkSumRx = checksum(toCheckRx, MSGLENGTH_HOME - 4);
	int checkSumMsg = getIntFromArray(buffer, MSGLENGTH_HOME - 4, 2);
	if (checkSumRx == checkSumMsg)
	{
		int latInt, lonInt, bearingInt, missionInt, geotetherRInt;
		double homeLat, homeLon;
		float homeBearing;
		latInt = getIntFromArray(buffer, 0, 4);
		lonInt = getIntFromArray(buffer, 4, 4);
		bearingInt = getIntFromArray(buffer, 8, 4);
		geotetherRInt = getIntFromArray(buffer, 12, 3);
		missionInt = getIntFromArray(buffer, 15, 3);
		homeLat = (double)latInt*0.000001;
		homeLon = (double)lonInt*0.000001;
		homeBearing = (float)bearingInt*0.01f;
		#ifdef DEBUG_PRINTF
			printf("Home Position - Lat: %.7f\t, Lon: %.7f\t, Bearing: %.2f\n", homeLat, homeLon, (double)homeBearing);
        #endif
		home.lat = homeLat;
		home.lon = homeLon;
		home.bearing = homeBearing;
		home.set = true;
		home.geotether_radius = geotetherRInt;
		home.mission_id = missionInt;
        if (formation_home_pub_fd == nullptr)
            formation_home_pub_fd = orb_advertise(ORB_ID(formation_home), &home);
        else
            orb_publish(ORB_ID(formation_home), formation_home_pub_fd, &home);
	}
}

int GroundComm::decode_fancy_landing_message(unsigned char *buffer)
{
    unsigned char toCheckRx[MSGLENGTH_FANCYLANDING - 4];
    memcpy(toCheckRx, &buffer[0], MSGLENGTH_FANCYLANDING - 4);
    int checkSumRx = checksum(toCheckRx, MSGLENGTH_FANCYLANDING - 4);
    int checkSumMsg = getIntFromArray(buffer, MSGLENGTH_FANCYLANDING - 4, 2);
    int log_state = -1;
    if (checkSumRx == checkSumMsg)
    {
        int XInt, YInt, altInt, acID, grpID, state;
        grpID = getIntFromArray(buffer, 0, 1);
        acID = getIntFromArray(buffer, 1, 1);
        state = getIntFromArray(buffer, 2, 1);
        altInt = getIntFromArray(buffer, 11, 1);
        if (state == 0)         //standard fancy landing
        {
            if (grpID == GROUP_ID)
            {
                returnHomeParam.z = (float)-altInt; //change sign for positive downwards
                returnHomeParam.acid = acID;
                returnHomeParam.grpid = grpID;
                if (return_to_home_parameters_pub_fd == nullptr)
                    return_to_home_parameters_pub_fd = orb_advertise(ORB_ID(return_to_home_parameters), &returnHomeParam);
                else
                    orb_publish(ORB_ID(return_to_home_parameters), return_to_home_parameters_pub_fd, &returnHomeParam);
#ifdef DEBUG_PRINTF
    printf("[gcomm] return to home published %.1f\n", (double)returnHomeParam.z);
#endif
                if (acID == AC_ID)          //if landing leader
                {
                    commandModes.aircraftID = acID;
                    commandModes.formation_mode = 4;
                    if(commanded_modes_pub_fd == nullptr)
                        commanded_modes_pub_fd = orb_advertise(ORB_ID(commanded_modes), &commandModes);
                    else
                        orb_publish(ORB_ID(commanded_modes), commanded_modes_pub_fd, &commandModes);
                    log_state = 8;
                }
            }
        }
        else if (state == 1)    //collision-free fancy landing
        {
            XInt = getIntFromArray(buffer, 3, 4);
            YInt = getIntFromArray(buffer, 7, 4);
            noFlyZone.x = (float)XInt*0.01f;
            noFlyZone.y = (float)YInt*0.01f;
            noFlyZone.acid = acID;
            noFlyZone.grpid = grpID;
            noFlyZone.z = (float)-altInt; //change sign for positive downwards
            if (no_fly_zone_parameters_pub_fd == nullptr)
                no_fly_zone_parameters_pub_fd = orb_advertise(ORB_ID(no_fly_zone_parameters), &noFlyZone);
            else
                orb_publish(ORB_ID(no_fly_zone_parameters), no_fly_zone_parameters_pub_fd, &noFlyZone);
#ifdef DEBUG_PRINTF
    printf("no flying zone published GRPID: %d\t ACID: %d\t X: %.7f\t Y: %.7f\n", grpID, acID, (double)noFlyZone.x, (double)noFlyZone.y);
#endif
            if (grpID == GROUP_ID && acID == AC_ID)                 //leader is landing
            {
                returnHomeParam.z = (float)-altInt; //change sign for positive downwards
                returnHomeParam.acid = acID;
                returnHomeParam.grpid = grpID;
                if (return_to_home_parameters_pub_fd == nullptr)
                    return_to_home_parameters_pub_fd = orb_advertise(ORB_ID(return_to_home_parameters), &returnHomeParam);
                else
                    orb_publish(ORB_ID(return_to_home_parameters), return_to_home_parameters_pub_fd, &returnHomeParam);
                #ifdef DEBUG_PRINTF
                    printf("return to home published %.1f\n", (double)returnHomeParam.z);
                #endif
                commandModes.aircraftID = acID;
                commandModes.formation_mode = 4;

            }
            else                    //another drone is landing, leader avoiding NFZ
            {

                commandModes.aircraftID = AC_ID;
                commandModes.formation_mode = 5;

            }
#ifdef DEBUG_PRINTF
    printf("[gcomm] publish commanded modes: acID: %d, formationmode: %d",commandModes.aircraftID,commandModes.formation_mode);
#endif
            if(commanded_modes_pub_fd == nullptr)
                commanded_modes_pub_fd = orb_advertise(ORB_ID(commanded_modes), &commandModes);
            else
                orb_publish(ORB_ID(commanded_modes), commanded_modes_pub_fd, &commandModes);
            log_state = 9;
        }

    }
    return log_state;
}

void GroundComm::decode_heartbeat_message(unsigned char *buffer)
{
	unsigned char toCheckRx[MSGLENGTH_HEARTBEAT - 4];
	memcpy(toCheckRx, &buffer[0], MSGLENGTH_HEARTBEAT - 4);
	int checkSumRx = checksum(toCheckRx, MSGLENGTH_HEARTBEAT - 4);
	int checkSumMsg = getIntFromArray(buffer, MSGLENGTH_HEARTBEAT - 4, 2);
	if (checkSumRx == checkSumMsg)
    {
        int time = getIntFromArray(buffer, 0, 3);
        int mode = buffer[3];
        if(mode==1)
        {
            polling.gsc_poll_requested++;
            polling.mission_time = (float)time*0.1f;    //time in seconds
            if (polling_request_pub_fd == nullptr)
                polling_request_pub_fd = orb_advertise(ORB_ID(polling_request), &polling);
            else
                orb_publish(ORB_ID(polling_request), polling_request_pub_fd, &polling);
            #ifdef DEBUG_PRINTF
                printf("heartbeat published %d\t %.1f\n", polling.gsc_poll_requested, (double)polling.mission_time);
            #endif
        }
    }
}

int GroundComm::decode_command_message(unsigned char *buffer)
{
	unsigned char toCheckRx[MSGLENGTH_COMMAND - 4];
	memcpy(toCheckRx, &buffer[0], MSGLENGTH_COMMAND - 4);
	int checkSumRx = checksum(toCheckRx, MSGLENGTH_COMMAND - 4);
	int checkSumMsg = getIntFromArray(buffer, MSGLENGTH_COMMAND - 4, 2);
	int command_log = -1;
	if (checkSumRx == checkSumMsg)
	{
		int acID, groupID, command_int;
		groupID = getIntFromArray(buffer, 0, 3);
		acID = getIntFromArray(buffer, 3, 3);
		command_int = buffer[6];
		armed_cmd = true;

		if (groupID == 0 || groupID == GROUP_ID)
		{
            command_log = command_int;
			if (command_int == 1 || command_int == 4)
			{
				if (command_int == 1)   //arm
				{
					commandArm.armed_status = true;
					commandArm.aircraftID = acID;
				}
				else if (command_int == 4)  //disarm
				{
					commandArm.armed_status = false;
					commandArm.aircraftID = acID;
				}


                if(commanded_arm_pub_fd == nullptr)
					commanded_arm_pub_fd = orb_advertise(ORB_ID(commanded_arm), &commandArm);
				else
					orb_publish(ORB_ID(commanded_arm), commanded_arm_pub_fd, &commandArm);
			}
			else
			{
				if (command_int == 2)   //auto formation
				{
					commandModes.formation_mode = 2;
					commandModes.aircraftID = acID;
				}
				else if (command_int == 3)  //stop formation
				{
					commandModes.formation_mode = 0;
					commandModes.aircraftID = acID;
				}
				else if (command_int == 5)      //reserved for return home and land
				{
					commandModes.formation_mode = 3;
					commandModes.aircraftID = acID;
				}
                else if (command_int == 6)      //contract - stop avoiding NFZ
                {
                    commandModes.formation_mode = 6;
                    commandModes.aircraftID = acID;
                    command_log = 10;
                }
                else if (command_int == 7)      //resume mission
                {
                    commandModes.formation_mode = 2;
                    commandModes.aircraftID = acID;
                    command_log = 11;
                }

                if(commanded_modes_pub_fd == nullptr)
                    commanded_modes_pub_fd = orb_advertise(ORB_ID(commanded_modes), &commandModes);
                else
                    orb_publish(ORB_ID(commanded_modes), commanded_modes_pub_fd, &commandModes);
            }
			#ifdef DEBUG_PRINTF
				printf("ground_comm command message published %d\n", command_int);
            #endif
		}
	}
	return command_log;
}

void GroundComm::decode_additional_cmd_message(unsigned char *buffer)
{
	unsigned char toCheckRx[MSGLENGTH_GCSCMD - 4];
	memcpy(toCheckRx, &buffer[0], MSGLENGTH_GCSCMD - 4);
	int checkSumRx = checksum(toCheckRx, MSGLENGTH_GCSCMD - 4);
	int checkSumMsg = getIntFromArray(buffer, MSGLENGTH_GCSCMD - 4, 2);
	if (checkSumRx == checkSumMsg)
	{
		int acID, groupID, command_int;
		groupID = getIntFromArray(buffer, 0, 3);
		acID = getIntFromArray(buffer, 3, 3);
		command_int = buffer[6];

		#ifdef DEBUG_PRINTF
			printf("Group: %d\t AC:%d\t Cmd:%d\n",groupID, acID, command_int);
        #endif
        if (groupID == 0 || groupID == GROUP_ID)	//To this group
        {
            publish_additional_commanded_modes(acID, command_int);
            if ((acID == 0 || acID == AC_ID) && command_int == 3)
            {
                #ifdef DEBUG_PRINTF
                    printf("Compass Calibration Command\n");
                #endif
                sensorCalib.cmd = sensorCalib.CALIBRATION_COMMAND_MAG;
                if (sensor_calibration_command_pub_fd == nullptr)
                    sensor_calibration_command_pub_fd = orb_advertise(ORB_ID(sensor_calibration_command), &sensorCalib);
                else
                    orb_publish(ORB_ID(sensor_calibration_command), sensor_calibration_command_pub_fd, &sensorCalib);
            }
		}
	}
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
	orb_check(vehicle_global_position_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(vehicle_global_position), vehicle_global_position_sub_fd, &global_pos);
	}
	orb_check(vehicle_control_mode_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(vehicle_control_mode), vehicle_control_mode_sub_fd, &ctl_mode);
		if (ctl_mode.flag_control_auto_formation_enabled)
			autoFormMode = true;
		else
			autoFormMode = false;
	}
    orb_check(sensor_calibration_status_sub_fd, &updated);
    if(updated)
    {
        orb_copy(ORB_ID(sensor_calibration_status), sensor_calibration_status_sub_fd, &calibStatus);
        if (calibStatus.state == sensor_calibration_status_s::CALIBRATION_STATUS_STATE_SLEEP ||
                calibStatus.state == sensor_calibration_status_s::CALIBRATION_STATUS_STATE_ON)
        {
            if ((calibStatus.status & sensor_calibration_status_s::CALIBRATION_STATUS_BITMASK_TURNING) &&
                    (calibStatus.status & sensor_calibration_status_s::CALIBRATION_STATUS_BITMASK_NEXT_SIDE))
            {
                compass_calibration_requested = false;
                compass_calibration_completed = true;
            }
            else
            {
                compass_calibration_requested = true;
                compass_calibration_completed = false;
            }
        }
        else
        {
            compass_calibration_requested = false;
            compass_calibration_completed = false;
        }
    }
	orb_check(mesh_network_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(mesh_network), mesh_network_sub_fd, &meshNetwork);
	}
	orb_check(vehicle_local_position_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub_fd, &local_pos);
	}
	orb_check(vehicle_gps_position_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub_fd, &gps_pos);
		if (gps_pos.fix_type > 2)	//3 and above = GPS fixed
			gpsStatus = true;
		else
			gpsStatus = false;
	}
	orb_check(battery_status_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(battery_status), battery_status_sub_fd, &battery);
	}
	orb_check(sensor_combined_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(sensor_combined), sensor_combined_sub_fd, &sensor);
        float sumsqr = 0.0f;
        for(int i=0;i<3;i++)
            sumsqr+=sensor.magnetometer_ga[i]*sensor.magnetometer_ga[i];
        float magnitude = (float) sqrt( sumsqr );

        float removed_value = total_mag_field_strength[mag_buffer_head];
        total_mag_field_strength[mag_buffer_head] = magnitude;
        mag_sum += magnitude - removed_value;
        mag_buffer_head++;
        if(mag_buffer_head>=MAG_BUFFER_LENGTH)
            mag_buffer_head=0;
	}
	orb_check(formation_cmdr_error_sub_fd, &updated);
	if(updated)
	{
		orb_copy(ORB_ID(formation_cmdr_error), formation_cmdr_error_sub_fd, &cmd_error);
	}
    orb_check(arrow_time_sub_fd, &updated);
	if (updated)
	{
        orb_copy(ORB_ID(arrow_time), arrow_time_sub_fd, &arrowTime);
	}
}

void GroundComm::updateAccZ(float accZ_curr)
{
	int accZInt = (int)accZ_curr*10;	//convert to integer * 10
	if (accZInt > max_accZ)
	{
		max_accZ = accZInt;
	}
	if (accZInt < min_accZ)
	{
		min_accZ = accZInt;
	}
}

int GroundComm::checkAlerts()		//0 = good, 1 = vibration, 2 = oscillation, 3 = home is not set, 4 - invalid mission ID, 5 - compass calibration requested, 6 - compass calibration completed
{
	int value = 0;
	int mean = -98;	//g * 10
	int limit = 30;
	int upper = mean + limit;
	int lower = mean - limit;
	if (max_accZ > upper || min_accZ < lower)
	{
		//evaluation finished, reset max and min values
		max_accZ = mean;
		min_accZ = mean;
		value = 1;
	}
	if (!home.set)
	{
		value = 3;
	}
    else if (cmd_error.error == formation_cmdr_error_s::MISSION_READ_ERROR)
	{
		value = 4;
	}  
    if (compass_calibration_requested)
    {
        value = 5;
    }
    if (compass_calibration_completed)
    {
        value = 6;
        compass_calibration_completed = false;
    }
	return value;
}

void GroundComm::getThreeByteArray(int num, unsigned char *array)
{
	//Masks out each Byte of number and assigns to character array
	array[2] = (num >> 16) & 0xFF;
	array[1] = (num >> 8) & 0xFF;
	array[0] = num & 0xFF;
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

void GroundComm::send_all_position(int groupID, int ownID)
{
	int i;
	//Send own position
	int status = vehicleStatus(gpsStatus, ctl_mode.flag_armed, autoFormMode);
    systemAlert = checkAlerts();
	
    double lat_to_send, lon_to_send;
#ifndef NEVER_SEND_RAW_GPS
    if (gps_pos.fix_type == 6)      //use raw GPS under RTK fix
    {
        lat_to_send = (double)(gps_pos.lat*0.0000001f);
        lon_to_send = (double)(gps_pos.lon*0.0000001f);
    }
    else                            //use estimated GPOS without RTK fix
#endif
    {
        lat_to_send = global_pos.lat;
        lon_to_send = global_pos.lon;
    }
    float mag_mean=mag_sum/MAG_BUFFER_LENGTH;
//printf("[gnd comm] sending mag_mean: %4.3f\n",(double)mag_mean);

    //send_group_pos_message(groupID, ownID, lat_to_send, lon_to_send, local_pos.z, status, gps_pos.fix_type, gps_pos.satellites_used, gps_pos.eph, battery.voltage_filtered_v, systemAlert);
    send_group_pos_message(groupID, ownID, lat_to_send, lon_to_send, mag_mean, status, gps_pos.fix_type, gps_pos.satellites_used, gps_pos.eph, battery.voltage_filtered_v, systemAlert);

	//Send group position
//	for (i = 1; i < MESHSIZE; i++)
//	{
//		send_group_pos_message(groupID, i+1, 0, 0, 0,0,0,0,0,0,0);
//		usleep(5000);
//	}
	for (i = 1; i < MESHSIZE; i++)
	{
		if (meshNetwork.acID[i] > 0)
		{
			send_group_pos_message(groupID, meshNetwork.acID[i], meshNetwork.lat[i], meshNetwork.lon[i], meshNetwork.alt[i], meshNetwork.formMode[i], meshNetwork.fixType[i], meshNetwork.nSat[i], meshNetwork.eph[i], meshNetwork.battV[i], meshNetwork.alert[i]);
			usleep(5000);		//delay 5ms
		}
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

int GroundComm::vehicleStatus(bool gps, bool armed, bool automode)
{
	if (!gps)
	{
		if (!armed)
		{
			if (!automode)
				return 0;
			else
				return 1;
		}
		else
		{
			if (!automode)
				return 2;
			else
				return 3;
		}
	}
	else
	{
		if (!armed)
		{
			if (!automode)
				return 4;
			else
				return 5;
		}
		else
		{
			if (!automode)
				return 6;
			else
				return 7;
		}
	}
}
