#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "st_fifo.h"

uint16_t out_slot_size;

int main(int argc, char *argv[])
{
	if (argc != 3) {
		printf("Error: please specify input and output as commandline args\n");
		return -1;
	}

	FILE *fp;
	char *line = NULL;
	fp = fopen(argv[1], "r");

	size_t len = 0;
	ssize_t read;

	FILE *out_file;
	out_file = fopen(argv[2], "w");

	int lines = 0;
	size_t pos = ftell(fp);
	while(!feof(fp)) {
		char ch = fgetc(fp);
		if(ch == '\n') {
			lines++;
		}
	}
	fseek(fp, pos, SEEK_SET);

	printf("File length: %d\n", lines);

	st_fifo_conf conf;
	conf.device = ST_FIFO_LSM6DSV16X;
	conf.bdr_xl = 0;
	conf.bdr_gy = 0;
	conf.bdr_vsens = 0;

	st_fifo_init(&conf);
	st_fifo_raw_slot *raw_slot;
	st_fifo_out_slot *out_slot;

	raw_slot = malloc(lines * sizeof(st_fifo_raw_slot));
	out_slot = malloc(lines * 3 * sizeof(st_fifo_out_slot));

	int i = 0;

	printf("Running test...\n");

	while((read = getline(&line, &len, fp)) != -1) {

		int data_in[7];

		sscanf(line, "%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", &data_in[0], &data_in[1], &data_in[2], &data_in[3], &data_in[4], &data_in[5], &data_in[6]);

		for (uint8_t j = 0; j < 7; j++)
			raw_slot[i].fifo_data_out[j] = data_in[j];

		i++;
  	}

	st_fifo_decode(out_slot, raw_slot, &out_slot_size, lines);
	st_fifo_sort(out_slot, out_slot_size);

	uint16_t acc_samples = st_fifo_get_sensor_occurrence(out_slot, out_slot_size, ST_FIFO_ACCELEROMETER);
	uint16_t gyr_samples = st_fifo_get_sensor_occurrence(out_slot, out_slot_size, ST_FIFO_GYROSCOPE);

	printf("acc samples: %d\n", acc_samples);
	printf("gyr samples: %d\n", gyr_samples);

	st_fifo_out_slot *acc_slot = malloc(acc_samples * sizeof(st_fifo_out_slot));
	st_fifo_out_slot *gyr_slot = malloc(gyr_samples * sizeof(st_fifo_out_slot));

	st_fifo_extract_sensor(acc_slot, out_slot, out_slot_size, ST_FIFO_ACCELEROMETER);
	st_fifo_extract_sensor(gyr_slot, out_slot, out_slot_size, ST_FIFO_GYROSCOPE);

	for (int i = 0; i < out_slot_size; i++) {
		fprintf(out_file, "%u\t%d\t%d\t%d\t%d\r\n", out_slot[i].timestamp, out_slot[i].sensor_tag, out_slot[i].sensor_data.data[0], out_slot[i].sensor_data.data[1], out_slot[i].sensor_data.data[2]);
	}
	/*
	for (int i = 0; i < acc_samples; i++) {
		printf("ACC SLOT:\t%lld\t%d\t%d\t%d\t%d\r\n", acc_slot[i].timestamp, acc_slot[i].sensor_tag, acc_slot[i].data[0], acc_slot[i].data[1], acc_slot[i].data[2]);
	}

	for (int i = 0; i < gyr_samples; i++) {
		printf("GYR SLOT:\t%lld\t%d\t%d\t%d\t%d\r\n", gyr_slot[i].timestamp, gyr_slot[i].sensor_tag, gyr_slot[i].data[0], gyr_slot[i].data[1], gyr_slot[i].data[2]);
	}
	*/

	printf("Test finished: see %s file\n", argv[2]);

  	fclose(fp);
  	fclose(out_file);

  	return 0;
}
