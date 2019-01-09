// respectlib.cpp: определяет экспортированные функции для приложения DLL.
//

#include "stdafx.h"
//#include <alloc.h>
//#include <mem.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


namespace respect_uc1
{
	class commands
	{


		/* Version symbols */
		#define UC1_DEVVER_1_00 0
		#define UC1_DEVVER_1_01 1

		/* Buffer sizes */
		#define DBUF_SIZE ((4096*4) + 100)
		#define COMMAND_BUFFER_SIZE 256
		#define REPLY_BUFFER_SIZE 1024
		#define TRANSCRIPT_SIZE 20

		/****************************************************************************
		*  Global device configuration data.
		****************************************************************************/
		/* The name for our device representation */
		static const char DevName[] = "UC1";

		/* Config profile section to store config data */
		static const char ConfigSection[] = "UC1 Hardware Configuration";

		/* Configuration */
		static const int config_loaded = 0;    /* We should read cfg only once per session */

										 /*
										 *  These arrays used to pass channel information with SPSRDevInfo struct.
										 *
										 *  XXX. We support only one controller with only one channel!
										 */
		static int channel_elements[1] = { 0 };
		static int channel_bits[1] = { 0 };

		/* Misc globals */
		static DWORD acq_start_time;  // For detection of timeouts.

		#define MAX_ACQ_WAIT_TIME (30*1000)


									  /*---------------------------------------------------------------------------
									  *  Version ID strings
									  */
		char *version_idstr[] =
		{
			"1.00",
			"1.01"
		};
		static int num_versions = sizeof(version_idstr) / sizeof(version_idstr[0]);

		/*---------------------------------------------------------------------------
		*  State variables
		*/
		static char* error_log_file_name = 0;
		static FILE * log_file = 0;


		HANDLE h_comm = INVALID_HANDLE_VALUE;
		int device_version = -1;

#define CONNECTED (h_comm != INVALID_HANDLE_VALUE)

		/* These data constitues the device configuration vars */
		short port_num = 0;     // COM port used to connect to device

		int range_min = 0;      // Min available position (0)
		int range_max = 0;      // Max available position

		int min_pos = 0;        // User defined min
		int max_pos = 0;        // User defined max

		int limsw1_ref = 0;   // Reference level for limit switch 1
		int limsw2_ref = 0;   // Reference level for limit switch 2

#define CHANNEL_ELEMENTS (range_max - range_min + 1)  //(max_pos - min_pos)

							  /* These are the device settings vars */
		int stepping_rate = 1000;
		int reversed_data_array = 0;
		int bidir_scanning = 1;

		/* This is the device state information cache */
		int current_pos = 0;
		int limsw1_level = 0;
		int limsw2_level = 0;
		int running = 0;

		/* Buffers */
		static long data_buffer[DBUF_SIZE];
		static char command_buffer[COMMAND_BUFFER_SIZE];
		static char reply_buffer[REPLY_BUFFER_SIZE];


		/*---------------------------------------------------------------------------
		*  Protocol transcript record
		*/
		typedef struct
		{
			DWORD time;
			int line;
			char* caller;
			int is_reply;
			char buffer[REPLY_BUFFER_SIZE];
		} transcript_record;
		static transcript_record transcript[TRANSCRIPT_SIZE];


		/****************************************************************************
		* Some utilities
		****************************************************************************/
#include <mmsystem.h>
		static DWORD
			elapsed_time(DWORD start)
		{
			DWORD cur = timeGetTime();
			if (cur >= start)
				return cur - start;
			else
			{
				/* Handle timer wrap around */
				return cur + (0xFFFFFFFF - start);
			}
		} /* elapsed_time(DWORD) */

		static DWORD
			time_diff(DWORD cur, DWORD prev)
		{
			if (cur >= prev)
				return cur - prev;
			else
			{
				/* Handle timer wrap around */
				return cur + (0xFFFFFFFF - prev);
			}
		} /* time_diff(DWORD cur, DWORD prev) */


		static char*
			get_last_error_msg(char* prefix)
		{
			static char msg_buf[1024];
			char* pos = msg_buf;

			if (prefix)
			{
				strcpy(msg_buf, prefix);
				pos += strlen(prefix);
			}

			FormatMessage(
				FORMAT_MESSAGE_FROM_SYSTEM,
				NULL,
				GetLastError(),
				MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
				(LPTSTR)pos,
				1023,
				NULL
			);

			return msg_buf;
		} /* get_last_error_msg(char* prefix) */


		  /****************************************************************************
		  *  Protocol transcript handlers
		  ****************************************************************************/
		static int first_index = -1;
		static int last_index = -1;

		static void
			put_in_transcript(int line, char* caller, int is_reply)
		{
			if (++last_index >= TRANSCRIPT_SIZE)
				last_index = 0;

			if (first_index < 0)
				first_index = 0;
			else if (first_index == last_index)
			{
				if (++first_index >= TRANSCRIPT_SIZE)
					first_index = 0;
			}

			transcript[last_index].time = timeGetTime();
			transcript[last_index].line = line;
			transcript[last_index].caller = caller;
			transcript[last_index].is_reply = is_reply;
			strcpy(transcript[last_index].buffer,
				is_reply ? reply_buffer : command_buffer);

		} /* put_in_transcript(int line, char* caller, int is_reply) */

		static void
			dump_transcript(FILE* log)
		{
			int cur;
			DWORD start;

			if (first_index >= 0 && last_index >= 0 && log)
			{
				cur = first_index;
				start = transcript[cur].time;

				fprintf(log, "Protocol transcript:\n");

				while (1)
				{
					fprintf(log,
						"dt = %6d ms: %s -> {%s}: Caller -> %s (line %d)\n",
						time_diff(transcript[cur].time, start),
						transcript[cur].is_reply ? "Reply " : "Command",
						transcript[cur].buffer,
						transcript[cur].caller,
						transcript[cur].line

					);

					if (cur == last_index)
						break;

					if (++cur >= TRANSCRIPT_SIZE)
						cur = 0;
				}

				fprintf(log, "\n\n");
			}
		} /* dump_transcript(FILE* log) */

		  /*---------------------------------------------------------------------------
		  *  Compute and setup error log file name.
		  */
#define PATHSIZE 1024
		void
			set_error_log_file_name(void)
		{
			static char name[PATHSIZE + 16];
			char* fnm;

			if (dev_hinst == INVALID_HANDLE_VALUE)
			{
				error_log_file_name = 0;
				return;
			}

			if (GetModuleFileName(dev_hinst, name, PATHSIZE))
			{
				fnm = name + strlen(name) - 1;

				while (fnm != name && *fnm != '\\' && *fnm != ':')
					fnm--;

				if (*fnm != '\\')
					fnm = name + strlen(name);

				strcpy(fnm, "\\errorlog.txt");
				error_log_file_name = name;
			}
		} /* set_error_log_file_name(void) */

		  /*-------------------------------------------------------------------------
		  *  Open/Close log file
		  *
		  *  This functions are called from PPD_Init()/PPD_Done() APIs.
		  */
		void
			open_error_log_file(void)
		{
			if (error_log_file_name)
				log_file = fopen(error_log_file_name, "wt");

		} /* open_error_log_file(void) */

		void
			close_error_log_file(void)
		{
			if (log_file)
			{
				fclose(log_file);
				log_file = 0;
			}
		} /* close_error_log_file(void) */

		  /*-------------------------------------------------------------------------
		  *  Write error log message.
		  */
		void
			write_error_log(int code, int line, char* msg)
		{
			if (log_file)
			{
				fprintf(log_file, "Firmware version %s.\n",
					(device_version >= 0 && device_version < num_versions) ?
					version_idstr[device_version] : "UNKNOWN");
				fprintf(log_file, "Error %d \"%s\" generated by %s (at line %d):\n",
					code, SPSRDEV_GetErrorMsg((SPSRDevStatus)code), msg, line);
				fprintf(log_file, "  reply_buffer -> {%s}\n\n", reply_buffer);
				dump_transcript(log_file);
				fclose(log_file);
				log_file = fopen(error_log_file_name, "at");
			}
		} /* write_error_log(int code, int line, char* msg) */


		  /****************************************************************************
		  *  Set error code in dev_status and write message to errorlog.txt file.
		  ****************************************************************************/
		void
			set_error_and_state(SPSRDevStatus err, int line, char* msg)
		{
			if (dev_status != err && err == spsrdevTimeout)
			{
				dev_status = err;
				//XXX report_com_errors_and_stat();
			}

			dev_status = err;

			if (dev_status != spsrdevOk && dev_status != spsrdevNoInit)
				write_error_log(err, line, msg);

		} /* set_error_and_state(PpdErrorCode err, int line, char* msg) */


		  /****************************************************************************
		  *  COM port handling stuff.
		  ****************************************************************************/
		  /*---------------------------------------------------------------------------
		  *  Opens serial communication port specified by portNumber argument,
		  *  sets proper communication parameters and timeouts values.
		  *  On success returns a HANDLE to opened communication port. On
		  *  failure returns INVALID_HANDLE_VALUE. GetLastError() can be used
		  *  to obtain detailed information about the failure. If GetLastError()
		  *  returns NO_ERROR then the failure is due to portNumber is out of range.
		  */
		static HANDLE
			open_comm_port(short portNumber)
		{
			HANDLE hComm;
			DCB dcb;
			COMMTIMEOUTS commTimeouts;
			char commName[10];
			DWORD comm_errors;
			COMSTAT  Stat;

			/*
			*  Check an argument is in range.
			*/
			if (portNumber < 1)
				return INVALID_HANDLE_VALUE;

			/*
			*  Patch the communication port name.
			*/
			sprintf(commName, "\\\\.\\COM%d", portNumber);

			/*
			*  Open a communication port.
			*/
			hComm = CreateFile(commName,
				GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, NULL);

			/*
			*  If open was successfull then setup port parameters.
			*/
			if (hComm != INVALID_HANDLE_VALUE)
			{
				if (!SetupComm(hComm, 16000, 16000))
				{
					CloseHandle(hComm);
					return INVALID_HANDLE_VALUE;
				}

				if (!GetCommState(hComm, &dcb))
				{
					CloseHandle(hComm);
					return INVALID_HANDLE_VALUE;
				}

				dcb.BaudRate = 115200;
				dcb.ByteSize = 8;
				dcb.StopBits = ONESTOPBIT;

				dcb.Parity = NOPARITY;
				dcb.fParity = FALSE;

				dcb.fOutxCtsFlow = FALSE; //TRUE;
				dcb.fRtsControl = RTS_CONTROL_DISABLE;

				dcb.fOutxDsrFlow = FALSE; //TRUE;
				dcb.fDtrControl = DTR_CONTROL_DISABLE;

				dcb.fOutX = FALSE;
				dcb.fInX = FALSE;

				dcb.fAbortOnError = FALSE;//TRUE;

				if (!SetCommState(hComm, &dcb))
				{
					CloseHandle(hComm);
					return INVALID_HANDLE_VALUE;
				}

				/*----------------------------------------------
				*  Setup communication timeouts
				*/

				/* ms allowed between each char */
				commTimeouts.ReadIntervalTimeout = 5000;

				/*
				*  Total read or write operation timeout calculated as:
				*  total = bytes_to_read*ReadTotalTimeoutMultiplier +
				*    ReadTotalTimeoutConstant;
				*/
				commTimeouts.ReadTotalTimeoutMultiplier = 200;
				commTimeouts.ReadTotalTimeoutConstant = 5000;

				commTimeouts.WriteTotalTimeoutMultiplier = 200;
				commTimeouts.WriteTotalTimeoutConstant = 5000;

				if (!SetCommTimeouts(hComm, &commTimeouts))
				{
					CloseHandle(hComm);
					return INVALID_HANDLE_VALUE;
				}

				PurgeComm(h_comm, PURGE_TXCLEAR);
				PurgeComm(h_comm, PURGE_RXCLEAR);
				ClearCommError(h_comm, &comm_errors, &Stat);
				Sleep(250);
			}

			return hComm;

		} /* open_comm_port(short) */


		  /*
		  *  Close serial communication port
		  */
		static void
			close_comm_port(HANDLE *hComm)
		{
			if (*hComm != INVALID_HANDLE_VALUE)
			{
				CloseHandle(*hComm);
				*hComm = INVALID_HANDLE_VALUE;
			}
		} /* close_comm_port(HANDLE *hComm) */


		  /*
		  *  Send a command message through a serial communication port.
		  *  Returns the number of bytes sent.
		  */
		static int
			comm_send(HANDLE hComm, const char* msg, int len)
		{
			DWORD writeLen = 0;

			WriteFile(hComm, msg, len, &writeLen, NULL);

			return writeLen;

		} /* comm_send(HANDLE, const char*, int) */


		  /*
		  *  Receive a message from a serial communication port.
		  *  Returns the number of bytes read.
		  */
		static int
			comm_receive(HANDLE hComm, char* msg, int len)
		{
			DWORD readLen = 0;

			SetLastError(0);
			ReadFile(hComm, msg, len, &readLen, NULL);

			return readLen;

		} /* comm_receive(HANDLE, char*, int) */


		  /*
		  *  Return number of bytes are in the input queue.
		  */
		static int
			bytes_pending(HANDLE hComm)
		{
			DWORD  Errors;
			COMSTAT  Stat;

			ClearCommError(hComm, &Errors, &Stat);

			return Stat.cbInQue;

		} /* bytes_pending(HANDLE hComm) */


		  /****************************************************************************
		  *  Device communication stuff.
		  ****************************************************************************/

		  /*
		  *  Open specified communication port and set state vars.
		  */
		void
			open_device(short port)
		{
			set_error_log_file_name();
			open_error_log_file();

			h_comm = open_comm_port(port_num = port);

			if (h_comm == INVALID_HANDLE_VALUE)
			{
				switch (GetLastError())
				{
				case NO_ERROR:
					set_error_and_state(spsrdevBadParams, __LINE__, "open_device:");
					break;

				case ERROR_FILE_NOT_FOUND:
					set_error_and_state(spsrdevNotInstalled, __LINE__, "open_device:");
					break;

				case ERROR_ACCESS_DENIED:
					set_error_and_state(spsrdevNoAccess, __LINE__, "open_device:");
					break;

				default:
					set_error_and_state(spsrdevNoInit, __LINE__, "open_device:");
				}
			}
			else
			{
				set_error_and_state(spsrdevOk, __LINE__, "open_device:");
			}
		} /* open_device(short) */


		  /*
		  *  Close opened port and set state vars.
		  */
		void
			close_device(void)
		{
			close_comm_port(&h_comm);
			set_error_and_state(spsrdevNoInit, __LINE__, "close_device:");
			close_error_log_file();
			//port_num = -1;
			device_version = -1;

		} /* close_device(void) */


		  /*
		  *  Clear input and output queues
		  */
		void
			purge_device(void)
		{
			DWORD comm_errors;

			if (h_comm != INVALID_HANDLE_VALUE)
			{
				/* Discard input and output buffers */
				PurgeComm(h_comm, PURGE_TXCLEAR | PURGE_TXABORT |
					PURGE_RXCLEAR | PURGE_RXABORT);
				//PurgeComm(h_comm, PURGE_RXCLEAR);

				ClearCommError(h_comm, &comm_errors, NULL);
			}
		} /* purge_device(void) */


		  /*
		  *  Return non zero if some bytes are in the input queue.
		  */
		static int
			device_reply_pending(void)
		{
			return  bytes_pending(h_comm) > 0;

		} /* reply_pending(void) */


		  /*
		  *  Send bytes in buf to device and add CR/LF terminator bytes.
		  */
		int
			send_device_cmd(char* buf, int line, char* caller)
		{
			char crlf[] = { 0x0D, 0x0A, 0x00 };
			int len = strlen(buf);

			if (h_comm != INVALID_HANDLE_VALUE)
			{
				if (!(comm_send(h_comm, buf, len) == len &&
					comm_send(h_comm, crlf, 1) == 1))
				{
					set_error_and_state(spsrdevTimeout, __LINE__,
						get_last_error_msg("send_device_cmd: WinError -> "));
					return 0;
				}

				put_in_transcript(line, caller, 0);

				return 1;
			}

			return 0;

		} /* send_device_cmd(char* buf, int line, char* caller) */


		  /*
		  *  Receive one line from device.
		  */
		int
			receive_device_reply_line(char* buf, int line, char* caller)
		{
			int n, cr, lf;
			char *anchor = buf;

			if (h_comm != INVALID_HANDLE_VALUE)
			{
				n = 0;
				cr = 0;
				lf = 0;

				memset(reply_buffer, 0, REPLY_BUFFER_SIZE);

				while (n < REPLY_BUFFER_SIZE - 10)
				{
					if (comm_receive(h_comm, buf, 1) != 1)
					{
						set_error_and_state(spsrdevTimeout, __LINE__,
							get_last_error_msg("receive_device_reply_line: WinError -> "));
						put_in_transcript(line, caller, 1);
						return 0;
					}

					if ((cr && *buf == 0x0A) || (lf && *buf == 0x0D))
					{
						/* Success! Put terminator at end */
						*(--buf) = 0;

						/* Trim the line */
						while (buf != anchor && *(--buf) == 32)
							*buf = 0;

						put_in_transcript(line, caller, 1);

						return 1;
					}

					if (*buf == 0x0D)
						cr = 1;
					else if (*buf == 0x0A)
						lf = 1;

					/* Convert to upper case */
					*buf = (char)toupper(*buf);

					n++;
					buf++;
				}

				set_error_and_state(spsrdevBadResponse, __LINE__, "receive_device_reply_line:");
				put_in_transcript(line, caller, 1);
			}

			return 0;

		} /* receive_device_reply_line(char* buf, int line, char* caller) */


		  /****************************************************************************
		  *  Device commands.
		  ****************************************************************************/

		  /*
		  *  Get device ID and set device_version. Set error state.
		  */
		int
			check_device_ID(void)
		{
			int res; // for debug
			int ver;
			char str[10];

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			device_version = -1; // Reset

			purge_device();
			strcpy(command_buffer, "id");

			res = send_device_cmd(command_buffer, __LINE__, "check_device_ID:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"check_device_ID:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"check_device_ID:");

			/* Parse result */
			if (res)
			{
				if (sscanf(reply_buffer, ">ID=UC1 FW %9s", str) != 1)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__, "check_device_ID:");
					device_version = -1;
					return 0;
				}

				for (ver = 0; ver < num_versions; ver++)
					if (strcmp(str, version_idstr[ver]) == 0)
					{
						device_version = ver;
						goto known_device;
					}

				set_error_and_state(spsrdevUnknownDevice, __LINE__, "check_device_ID:");
				return 0;

			known_device:
				return 1;
			}

			return 0;

		} /* check_device_ID(void) */

		  /*
		  *  Retreive motor position from the device and store to internal
		  *  cache var.
		  */
		int
			get_current_position_nocache(void)
		{
			int res; // for debug
			int pos;

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			strcpy(command_buffer, "position");

			res = send_device_cmd(command_buffer, __LINE__, "get_current_position_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_current_position_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_current_position_nocache:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">POS %d", &pos) != 1)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"get_current_position_nocache:");
					return 0;
				}

				return current_pos = pos;
			}

			return 0;

		} /* get_current_position_nocache(void) */



		  /*
		  *  Retreive motor motion range from the device and store to internal
		  *  cache var.
		  */
		int
			get_motion_range_nocache(void)
		{
			int res; // for debug
			int ra;

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			strcpy(command_buffer, "range");

			res = send_device_cmd(command_buffer, __LINE__, "get_motion_range_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_motion_range_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_motion_range_nocache:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">RANGE %d", &ra) != 1)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"get_motion_range_nocache:");
					return 0;
				}

				if (ra > 0)
					range_max = ra / 16 * 16 - 1;

				return range_max;
			}

			return 0;

		} /* get_motion_range_nocache(void) */


		  /*
		  *  Retreive motor stepping rate from the device and store to internal
		  *  cache var.
		  */
		int
			get_stepping_rate_nocache(void)
		{
			int res; // for debug
			int sp;

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			strcpy(command_buffer, "speed");

			res = send_device_cmd(command_buffer, __LINE__, "get_stepping_rate_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_stepping_rate_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_stepping_rate_nocache:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">SPEED %d", &sp) != 1)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"get_stepping_rate_nocache:");
					return 0;
				}

				return stepping_rate = sp;
			}

			return 0;

		} /* get_stepping_rate_nocache(void) */


		  /*
		  *  Set motor stepping rate. Return actually set value.
		  */
		int
			set_stepping_rate(int Hz)
		{
			int res; // for debug

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			sprintf(command_buffer, "speed %d", Hz);

			res = send_device_cmd(command_buffer, __LINE__, "set_stepping_rate:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"set_stepping_rate:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"set_stepping_rate:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">SPEED %d", &Hz) != 1)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"set_stepping_rate:");
					return 0;
				}

				return stepping_rate = Hz;
			}

			return 0;

		} /* set_stepping_rate(int Hz) */


		  /*
		  *  Retreive limit switch state from the device and store to internal
		  *  cache vars.
		  */
		int
			get_limsw_state_nocache(int sw)
		{
			int res; // for debug
			int level, ref;

			if (dev_status != spsrdevOk || sw < 1 || sw > 2)
				return 0; // Port not initialized or error state.

			purge_device();
			sprintf(command_buffer, "es%d", sw);

			res = send_device_cmd(command_buffer, __LINE__, "get_limsw_state_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_limsw_state_nocache:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"get_limsw_state_nocache:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">ES%d/REF %d/%d", &sw, &level, &ref) != 3)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"get_limsw_state_nocache:");
					return 0;
				}

				if (sw == 1)
				{
					limsw1_level = level;
					limsw1_ref = ref;
				}
				else
				{
					limsw2_level = level;
					limsw2_ref = ref;
				}

				return 1;
			}

			return 0;

		} /* get_limsw_state_nocache(int sw) */


		  /*
		  *  Set limit switch reference level. Return actually set value.
		  */
		int
			set_limsw_ref(int sw, int ref)
		{
			int res; // for debug
			int level;

			if (dev_status != spsrdevOk || sw < 1 || sw > 2)
				return 0; // Port not initialized or error state.

			purge_device();
			sprintf(command_buffer, "es%d %d", sw, ref);

			res = send_device_cmd(command_buffer, __LINE__, "set_limsw_ref:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"set_limsw_ref:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"set_limsw_ref:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (sscanf(reply_buffer, ">ES%d/REF %d/%d", &sw, &level, &ref) != 3)
				{
					set_error_and_state(spsrdevBadResponse, __LINE__,
						"set_limsw_ref:");
					return 0;
				}

				if (sw == 1)
				{
					limsw1_level = level;
					limsw1_ref = ref;
				}
				else
				{
					limsw2_level = level;
					limsw2_ref = ref;
				}

				return 1;
			}

			return 0;

		} /* set_limsw_ref(int sw, int ref) */


		  /*
		  *  Clear device onboard data memory in specified range.
		  */
		int
			clear_onboard_memory(int from, int to)
		{
			int res; // for debug

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or device in error state.

			purge_device();
			sprintf(command_buffer, "clear %d %d", from, to);

			res = send_device_cmd(command_buffer, __LINE__, "clear_onboard_memory:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"clear_onboard_memory:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"clear_onboard_memory:");

			/* Parse result */
			if (res && dev_status == spsrdevOk)
			{
				if (strcmp(reply_buffer, ">INVALID ARGUMENTS") == 0)
					set_error_and_state(spsrdevBadParams, __LINE__,
						"clear_onboard_memory:");
				else
				{
					sprintf(command_buffer, ">CLEAR FROM %d TO %d", from, to);
					if (strcmp(reply_buffer, command_buffer))
					{
						set_error_and_state(spsrdevBadResponse, __LINE__,
							"clear_onboard_memory:");
					}
				}
			}

			return dev_status == spsrdevOk;

		} /* clear_onboard_memory(int from, int to) */


		  /*
		  *  Read device onboard data memory to local buffer.
		  */
		int
			read_onboard_memory(int from, int to)
		{
			int res; // for debug
			int count;
			char* str;
			char* eptr;
			long lnum;
			long* buf = reversed_data_array ? data_buffer + to : data_buffer + from;

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or device in error state.

			purge_device();
			sprintf(command_buffer, "view %d %d", from, to);

			res = send_device_cmd(command_buffer, __LINE__, "read_onboard_memory:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"read_onboard_memory:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"read_onboard_memory:");

			/* Parse initial reply */
			if (res && dev_status == spsrdevOk)
			{
				if (strcmp(reply_buffer, ">INVALID ARGUMENTS") == 0)
				{
					set_error_and_state(spsrdevBadParams, __LINE__,
						"read_onboard_memory:");
					return 0;
				}
				else
				{
					sprintf(command_buffer, ">VIEW FROM %d TO %d", from, to);
					if (strcmp(reply_buffer, command_buffer))
					{
						set_error_and_state(spsrdevBadResponse, __LINE__,
							"read_onboard_memory:");
						return 0;
					}
				}
			}

			/* The data rows follows */
			count = to - from + 1;

			while (count &&
				receive_device_reply_line(reply_buffer, __LINE__,
					"read_onboard_memory:") &&
				dev_status == spsrdevOk)
			{
				str = reply_buffer;

				do
				{
					lnum = strtol(str, &eptr, 10);

					if (eptr && *eptr != 32 && *eptr != 0)
					{
						set_error_and_state(spsrdevBadResponse, __LINE__,
							"read_onboard_memory:");
						return 0;
					}

					if (reversed_data_array)
						*buf-- = lnum;
					else
						*buf++ = lnum;

					str = eptr;
					count--;
				} while (count && eptr && *eptr == 32);

				if (count)
				{
					// XXX for DEBUG
				}

			} // while(count &&

			return dev_status == spsrdevOk;

		} /* read_onboard_memory(int from, int to) */


		  /*
		  *  Check motion status.
		  */
		int
			is_in_motion(void)
		{
			int res;

			if (dev_status != spsrdevOk && dev_status != spsrdevBusy)
				return running = 0; // Port not initialized or device in error state.

									/* Check motion state */
			if (running && device_reply_pending())
			{
				running = 0;

				res = receive_device_reply_line(reply_buffer, __LINE__, "is_in_motion:");

				/* Parse result */
				if (res && (dev_status == spsrdevOk || dev_status == spsrdevBusy))
				{
					if (!(strstr(reply_buffer, "POS") || strstr(reply_buffer, "RANGE")))
					{
						set_error_and_state(spsrdevBadResponse, __LINE__,
							"is_in_motion:");
					}
					else
					{
						/* Update local cache */
						dev_status = spsrdevOk;
						get_current_position_nocache();
						get_motion_range_nocache();
					}
				}
			}

			return running;

		} /* is_in_motion(void) */


		  /*
		  *  Stop long term operation.
		  */
		int
			stop_device(void)
		{
			int res; // for debug

			if (!is_in_motion())
				return 1;

			purge_device();
			command_buffer[0] = 0x03;
			command_buffer[1] = 0x00;

			res = send_device_cmd(command_buffer, __LINE__, "stop_device:");
			running = 0;

			if (res && (dev_status == spsrdevOk || dev_status == spsrdevBusy))
			{
				/* Update local cache */
				Sleep(300);
				dev_status = spsrdevOk;
				get_current_position_nocache();
				get_motion_range_nocache();
			}

			return res;

		} /* stop_device(void) */


		  /*
		  *  Start range calibration process
		  */
		int
			start_range_calibration(int inipos)
		{
			int res; // for debug

			stop_device();

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			sprintf(command_buffer, "init %d", inipos);

			res = send_device_cmd(command_buffer, __LINE__, "start_range_calibration:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"start_range_calibration:");

			return running = (res && dev_status == spsrdevOk);

		} /* start_range_calibration(int inipos) */


		  /*
		  *  Start motion to specified position
		  */
		int
			start_motion(int pos)
		{
			int res; // for debug

			stop_device();

			if (dev_status != spsrdevOk)
				return 0; // Port not initialized or error state.

			purge_device();
			sprintf(command_buffer, "position %d", pos);

			res = send_device_cmd(command_buffer, __LINE__, "start_motion:");
			res = res && receive_device_reply_line(reply_buffer, __LINE__,
				"start_motion:");

			return running = (res && dev_status == spsrdevOk);

		} /* start_motion(int pos) */



		  /****************************************************************************
		  *  Configuration management stuff.
		  ****************************************************************************
		  *
		  *  Functions with names precided by underscore must not be called directly
		  *  from routines implementing device driver standard stuff.
		  */

		static void
			_set_default_params(void)
		{
			stepping_rate = 1000;
			reversed_data_array = 0;
			bidir_scanning = 1;

		} /* _set_default_params(void) */


		static void
			_clear_globals(void)
		{
			config_loaded = 0;
			_set_default_params();

			port_num = 0;

			if (h_comm != INVALID_HANDLE_VALUE)
				::CloseHandle(h_comm);

			h_comm = INVALID_HANDLE_VALUE;
			device_version = -1;

			for (int i = 0; i < DBUF_SIZE; i++)
				data_buffer[i] = 0;

			/* Clear device information */
			channel_elements[0] = 0;
			channel_bits[0] = 0;

		} /* _clear_globals(bool initial) */


		  /*-------------------------------------------------------------------------
		  * Access to configuration and parameters profile.
		  */

		  /* Schedule recongiguration by clearing config profile */
		static void
			shedule_reconfiguration(void)
		{
			erase_config_section(ConfigSection);

		} /* shedule_reconfiguration(void) */


		  /* Load configuration from profile */
		static void
			_load_config(void)
		{
			/* This operation should be performed only once per session */
			if (!config_loaded)
			{
				_clear_globals();

				port_num = (short)read_config_int(ConfigSection, "ComPort", 0);
				range_min = read_config_int(ConfigSection, "RangeMin", 0);
				range_max = read_config_int(ConfigSection, "RangeMax", 0); //range_max);
				min_pos = read_config_int(ConfigSection, "MinPos", 0); //min_pos);
				max_pos = read_config_int(ConfigSection, "MaxPos", 0); //max_pos);
				limsw1_ref = read_config_int(ConfigSection, "LimitSW1Ref", 0);
				limsw2_ref = read_config_int(ConfigSection, "LimitSW2Ref", 0);

				if (min_pos < range_min)
					min_pos = range_min;

				if (max_pos > range_max)
					max_pos = range_max;

				/* XXX. Here we must check the order and limits! */

				config_loaded = 1; /* Set flag that we have done */
			}
		} /* _load_config(void) */


		static void
			_store_config(void)
		{
			shedule_reconfiguration();  // Clear profile

			write_config_int(ConfigSection, "ComPort", port_num, 0);
			write_config_int(ConfigSection, "MinPos", min_pos, 0);
			write_config_int(ConfigSection, "MaxPos", max_pos, 0);
			write_config_int(ConfigSection, "RangeMin", range_min, 0);
			write_config_int(ConfigSection, "RangeMax", range_max, 0);
			write_config_int(ConfigSection, "LimitSW1Ref", limsw1_ref, 0);
			write_config_int(ConfigSection, "LimitSW2Ref", limsw2_ref, 0);

		} /* _store_config(void) */


		static void
			load_params(void)
		{
			stepping_rate = read_params_int("SteppingRate", stepping_rate);
			reversed_data_array = read_params_int("ReversedArray", reversed_data_array);
			bidir_scanning = read_params_int("BiDirScanning", bidir_scanning);
			current_pos = read_params_int("CurrentPos", current_pos);

			min_pos = read_params_int("MinPos", min_pos);
			max_pos = read_params_int("MaxPos", max_pos);

		} /* load_params(void) */

		static void
			store_params(void)
		{
			erase_params_section();

			write_params_int("SteppingRate", stepping_rate, 0);
			write_params_int("ReversedArray", reversed_data_array, 0);
			write_params_int("BiDirScanning", bidir_scanning, 0);
			write_params_int("CurrentPos", current_pos, 0);

			write_params_int("MinPos", min_pos, 0);
			write_params_int("MaxPos", max_pos, 0);

		} /* store_params(void) */


		  /*--------------------------------------------------------------------------*
		  *  Hardware detection/Config verification.
		  *--------------------------------------------------------------------------*/

		  //static bool hardware_detect_done = false;

		static bool
			hardware_init(void)
		{
			if (port_num <= 0)
				return false; /* Not configured */

			if (h_comm != INVALID_HANDLE_VALUE)
				return true;  /* Already initialized */

							  /* Open connection and check ID */
			open_device(port_num);
			check_device_ID();

			if (dev_status != spsrdevOk)
			{
				close_device();
				return false;
			}

			return true;

		} /* hardware_init(void) */


		  /*---------------------------------------------------------------------------
		  *  This function performs verification of hardware configuration.
		  *
		  *  NOTE: This function gets called in the beginning of the initialization
		  */
		static bool
			is_valid_configuration(void)
		{
			if (port_num <= 0)
				return false; /* Not configured */

			if (!hardware_init())
				return false; /* Not connected/powered or some init error. */

			return true;

		} /* is_valid_configuration(void) */


		  /*---------------------------------------------------------------------------
		  *  This function uses wizard-like UC1configDlg to setup internal
		  *  configuration data and write out this configuration to driver's profile.
		  *
		  *  If no sutable hardware was found or user has canceled configuration
		  *  setup dialog the function returns false. Othervise returns true.
		  */
		static bool
			_setup_configuration(void)
		{
			if (port_num > 0)
				return true; // Already configured

							 /* Invoke UC1configDlg and wait for user to select */
			if (!UC1Configure())
			{
				/* Configuration canceled. */
				MessageDlg(
					AnsiString(DevName) + ":\n" +
					"Unable to configure driver!\n"
					"Configuration procedure was not completed.\n"
					"Restart software to try again.",
					mtError, TMsgDlgButtons() << mbOK, 0);

				return false;
			}

			/* Write out config to profile */
			_store_config();

			close_device();

			return true;

		} /* _setup_configuration(void) */


		  /*---------------------------------------------------------------------------
		  *  Central point for configuration management.
		  */
		static bool
			is_configured(void)
		{
			static bool cfg_loaded = false;
			static bool cfg_done = false;

			if (!cfg_loaded)
			{
				cfg_loaded = true;
				_load_config();
			}

			if (port_num <= 0)
			{
				if (cfg_done)
					return false;

				cfg_done = true;

				return _setup_configuration();
			}

			return true;

		} /* is_configured(void) */


		  /****************************************************************************
		  *  Implement device specific layer of SpectraSource driver.
		  ****************************************************************************/

		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void init_device_info(SPSRDevInfo* info);
		  *
		  *  Description:
		  *    Initialize device information structure. Use profile to retrieve
		  *    hardware configuration settings.
		  */
		void
			init_device_info(SPSRDevInfo* info)
		{
			if (!is_configured())
				return;

			/* XXX. May be this is to be done in some other place? */
			channel_elements[0] = CHANNEL_ELEMENTS;
			channel_bits[0] = 12;

			/* Initialize device dependent fields */
			info->dev_name = DevName;
			info->n_channels = 1;
			info->channel_elements = channel_elements;
			info->channel_bits = channel_bits;

			/* Notify that we are able to change configuration  */
			info->caps |= SPSR_DEVINFO_CAP_RECONFIG;

		} /* init_device_info(SPSRDevInfo* info) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void init_device(void);
		  *
		  *  Description:
		  *    Initialize hardware to default state. Use profile to retrieve
		  *    user configurable settings. Sets dev_status to reflect result.
		  */
		void
			init_device(void)
		{
			/* XXX ToDo. Clear buffers */
			//setmem();

			/*
			*  This function called only after internal configuration information
			*  was successfuly setup. But there is no guaranty that physical hardware
			*  configuration corresponds to internal settings if these settings was
			*  loaded from configuration profile. So, we must do the test. Note: the
			*  COM-port is open if this function returns true.
			*/
			if (!is_valid_configuration())
			{
				if (dev_status != spsrdevBadResponse)
					dev_status = spsrdevCfgChanged; // Signal that hardware config is changed.
				return;
			}

			/*
			*  Get operational params saved in params profile.
			*/
			load_params();

			device_update_params();

			if (current_pos)
			{
				// The device was not parked. We need recalibration.
				start_range_calibration(0);

				/* Invoke UC1CancelDlg and wait */
				bool done = WaitForFinish(
					"Running device range calibration. Please wait...");

				if (dev_status != spsrdevOk)
					return;

				if (!done)
				{
					dev_status = spsrdevInitFailed;
					return;
				}
			}

			dev_status = spsrdevOk;

			// Final result will be set in the next call.
			// It is safe to call it here since worker thread is sleeping now.
			// XXX. Should we do it here??? May be we should use:
			//   if(acquire_params_lock()) release_params_lock(1);
			// and let the worker thread to do the rest?
			// device_update_params();

			if (acquire_params_lock()) release_params_lock(1);

		} /* init_device(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void done_device(void);
		  *
		  *  Description:
		  *    Shutdown device and store user configurable settings to profile.
		  */
		void
			done_device(void)
		{
			stop_device();

			/* Park the device to zero position */
			get_current_position_nocache();

			if (current_pos)
			{
				start_motion(0);
				WaitForFinish("Parking device. Please wait...");
			}

			/* Shutdown dev_status */
			dev_status = spsrdevNoInit;

			/* Store settings */
			store_params();

			/* Close connection */
			close_device();
			//close_error_log_file(); //XXX this already done in close_device() 

		} /* done_device(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void reset_device(void);
		  *
		  *  Description:
		  *    Reinitialize device using current settings. Set dev_status to reflect
		  *    result.
		  */
		void
			reset_device(void)
		{
			done_device();
			init_device();

			if (dev_status == spsrdevOk)
			{
				start_range_calibration(0);

				/* Invoke UC1CancelDlg and wait */
				bool done = WaitForFinish(
					"Running device range calibration. Please wait...");

				if (dev_status != spsrdevOk)
					return;

				if (!done)
				{
					dev_status = spsrdevInitFailed;
					return;
				}
			}
		} /* reset_device(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    int  device_reconfiguration(void);
		  *
		  *  Description:
		  *    Performs hardware reconfiguration. Returns nonzero if configuration
		  *    was changed (or sheduled to be changed to the next startup).
		  */
		int
			device_reconfiguration(void)
		{
			shedule_reconfiguration();
			return 1;

		} /* device_reconfiguration(void) */

		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_setup_dialog(HWND hParent);
		  *
		  *  Description:
		  *    Displays dialog box with device specific user configurable settings.
		  */
		void
			device_setup_dialog(HWND hParent)
		{
			if (!CONNECTED)
				return;

			UC1Setup();

		} /* device_setup_dialog(HWND hParent) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_update_params(void);
		  *
		  *  Description:
		  *    Upload parameters to the device (called when some parameter
		  *    has been changed). Set dev_status to reflect the result. Can
		  *    store successfully realized settings to profile.
		  */
		void
			device_update_params(void)
		{
			if (!CONNECTED)
				return;

			set_stepping_rate(stepping_rate);
			set_limsw_ref(1, limsw1_ref);
			set_limsw_ref(2, limsw2_ref);

		} /* device_update_params(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_begin_syncin(void);
		  *
		  *  Description:
		  *    Called to enable device to detect external triggering event (sync in).
		  */
		void
			device_begin_syncin(void)
		{
			// This is NOP for this devices.

		} /* device_begin_syncin(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_end_syncin(void);
		  *
		  *  Description:
		  *    Called after external triggering event has been detected to
		  *    disable sync-in sensing.
		  */
		void
			device_end_syncin(void)
		{
			// This is NOP for this devices.

		} /* device_end_syncin(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    int  device_syncin_detected(void);
		  *
		  *  Description:
		  *    Called to query external triggering event. Must return non-zero
		  *    if detected.
		  */
		int
			device_syncin_detected(void)
		{
			// This is NOP for this devices. External sync (if supported)
			// is handled by the hardware.
			return 1;

		} /* device_syncin_detected(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_start_acquisition(void);
		  *
		  *  Description:
		  *    Starts the data acquizition cycle. We must set dev_status to
		  *    spsrdevBusy to signal that we have success starting acquisition.
		  */
		static int prescan = 0;  // Flag set in initial phase of daq
		static int end_pos, start_pos;      // Start/End positions for work scan

		void
			device_start_acquisition(void)
		{
			int mid_pos;

			if (!CONNECTED)
				return;

			// XXX. Force to recheck configuration since controllers
			//      can be swaped or moved to other slots.
			//device_update_params();

			// Choose start/end positions
			if (bidir_scanning)
			{
				mid_pos = (min_pos + max_pos) / 2;

				if (current_pos <= mid_pos)
				{
					start_pos = min_pos;
					end_pos = max_pos;
				}
				else
				{
					start_pos = max_pos;
					end_pos = min_pos;
				}
			}
			else
			{
				start_pos = min_pos;
				end_pos = max_pos;
			}

			//clear_onboard_memory(0, 4095);

			// Try to initialize data acquisition.
			if (!start_motion(start_pos))
				return;

			prescan = 1;

			// If we get to here then all seems to be OK.
			acq_start_time = timeGetTime();
			dev_status = spsrdevBusy;

		} /* device_start_acquisition(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_stop_acquisition(void);
		  *
		  *  Description:
		  *    Prematurally terminates the data acquizition cycle.
		  */
		void
			device_stop_acquisition(void)
		{
			if (!CONNECTED)
				return;

			stop_device();

			// Restore ready status
			if (dev_status == spsrdevBusy)
				dev_status = spsrdevOk;

			prescan = 0;

		} /* device_stop_acquisition(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    void device_check_acquisition(void);
		  *
		  *  Description:
		  *    Checks the data acquizition state and if finished copies acquired
		  *    data to internal buffers and resets dev_status to whatever relevant.
		  *    dev_status other than spsrdevBusy signals to worker thread
		  *    that the acquisition has been completed
		  */
		void
			device_check_acquisition(void)
		{
			if (!CONNECTED)
				return;

			if (is_in_motion())
			{
#ifdef DETECT_TIMEOUTS
				if (elapsed_time(acq_start_time) > MAX_ACQ_WAIT_TIME)
				{
					// Timed out acquisition
					device_stop_acquisition();

					if (dev_status == spsrdevBusy)
						dev_status = spsrdevTimeout;
				}
#endif
				return;
			}

			if (prescan)
			{
				// Prescan finished. Start work scanning.
				prescan = 0;
				dev_status = spsrdevOk;

				if (clear_onboard_memory(0, 4095) && start_motion(end_pos))
				{
					// If we get to here then all seems to be OK.
					acq_start_time = timeGetTime();
					dev_status = spsrdevBusy; // Restore Busy status
				}
			}
			else
			{
				// Daq finished. Get results.
				// XXX Does not works. setmem((void*)data_buffer, DBUF_SIZE*sizeof(data_buffer[0]), 0);
				for (int i = 0; i < DBUF_SIZE; i++)
					data_buffer[i] = 0;

				if (dev_status == spsrdevOk)
					read_onboard_memory(min_pos, max_pos);
			}
		} /* device_check_acquisition(void) */


		  /*---------------------------------------------------------------------------
		  *  Function:
		  *    int  device_get_channel_data(int channel, double* buf);
		  *
		  *  Description:
		  *    Data exchange handler.
		  */
		int
			device_get_channel_data(int channel, double* buf)
		{
			int i;

			if (channel != 0)
				return 0;

			for (i = 0; i < channel_elements[0]; i++)
				buf[i] = data_buffer[i /* + min_pos */];

			return channel_elements[0];

		} /* device_get_channel_data(int channel, double* buf) */

		void
			device_set_roi(int channel, int start, int stop)
		{
			if (channel == 0)
			{
				min_pos = start;
				max_pos = stop;
			}
		} /* device_set_roi(int channel, int start, int stop) */


		  /*--EOF OrminsCCDdrv.c--*/





	};
}
