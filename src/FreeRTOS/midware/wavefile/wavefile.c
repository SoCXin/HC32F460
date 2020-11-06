#include "wavefile.h"

void wavefileinfo_init(WaveHeader* wavehead)
{
	wavehead->riff.ChunkID = 0X46464952;//"RIFF"
	wavehead->riff.ChunkSize = 0;//最后计算
	wavehead->riff.Format = 0X45564157;//"WAVE"
	wavehead->fmt.ChunkID = 0X20746D66;//"fmt";
	wavehead->fmt.ChunkSize = 16;
	wavehead->fmt.AudioFormat = 0x01;//0X01,表示PCM
	wavehead->fmt.NumOfChannels = 2;
	wavehead->fmt.SampleRate = 8000;//8KHZ采样率
	wavehead->fmt.ByteRate = wavehead->fmt.SampleRate*2*wavehead->fmt.NumOfChannels;//16位数据；
	wavehead->fmt.BlockAlign = 2;//16位为两个字节
	wavehead->fmt.BitsPerSample = 16;//16位；
	wavehead->data.ChunkID = 0X61746164;//"DATA";
	wavehead->data.ChunkSize = 0;//最后计算
}


