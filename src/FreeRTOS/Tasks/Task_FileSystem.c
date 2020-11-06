#include "../Tasks/Task_FileSystem.h"
FATFS FatFs;
FRESULT fr;
FIL Myfile;
FIL f_Wave;
uint32_t bw,len;
WaveHeader Wavedata;
char line[512];
TaskHandle_t H_Task_Fs;
//QueueHandle_t H_xQueue_Recode = NULL;

AudioBuff_type *pwavdata;
#define STACKSIZE_FS	configMINIMAL_STACK_SIZE
#define PRIORITY_TASKFS	(tskIDLE_PRIORITY+3)
static void Task_FS_Operation(void* param)
{
//    H_xQueue_Recode = xQueueGenericCreate(2,sizeof(AudioBuff_type),queueQUEUE_TYPE_BASE);
	hd_sdio_hw_init();
	disk_initialize(SD_Card);
	wavefileinfo_init(&Wavedata);
    f_mount(SD_Card,&FatFs);//Çý¶¯Æ÷0
    fr = f_open(&Myfile,"myfile.wav",FA_READ|FA_WRITE|FA_CREATE_ALWAYS);
	fr = f_write(&Myfile,&Wavedata,sizeof(Wavedata),&bw);
    
	fr = f_write(&Myfile,&au16PixieDustSoundI2s_8,u32WavLen_8k,&bw);
	Wavedata.data.ChunkSize = u32WavLen_8k;
	Wavedata.riff.ChunkSize = Wavedata.data.ChunkSize+36;
	f_lseek(&Myfile,0);
	fr = f_write(&Myfile,&Wavedata,sizeof(Wavedata),&bw);
	f_close(&Myfile);
	vTaskDelete(H_Task_Fs);
	while(1)
		{

//            fr = f_write(&Myfile,pwavdata,sizeof(AudioBuff_type),&bw);
//            Wavedata.data.ChunkSize += sizeof(AudioBuff_type);
//            Wavedata.riff.ChunkSize = Wavedata.data.ChunkSize+36;
//            if(Wavedata.data.ChunkSize>500000)
//            {
//                f_lseek(&Myfile,0);
//               	fr = f_write(&Myfile,&Wavedata,sizeof(Wavedata),&bw);
//                f_close(&Myfile);
//                vTaskDelete(H_Task_Fs);
//            }
		}
}


uint8_t Fs_Task_Start(void)
{
	xTaskCreate(Task_FS_Operation,(const char *)"File System Task",STACKSIZE_FS,NULL,PRIORITY_TASKFS,&H_Task_Fs);
	return Ok;
}























